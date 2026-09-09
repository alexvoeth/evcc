package charger

// LICENSE

// Copyright (c) evcc.io (andig, naltatis, premultiply)

// This module is NOT covered by the MIT license. All rights reserved.

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

import (
	"context"
	"encoding/hex"
	"fmt"
	"math"
	"strings"
	"time"

	"github.com/evcc-io/evcc/api"
	"github.com/evcc-io/evcc/api/implement"
	"github.com/evcc-io/evcc/util"
	"github.com/evcc-io/evcc/util/modbus"
	"github.com/volkszaehler/mbmd/encoding"
)

// SungrowDC is a Sungrow DC charger (IDC30E, IDC180E, IDC480E) speaking the
// "EMS and EV Charger MODBUS" protocol. The device only accepts a power
// setpoint per connector (gun); evcc current requests are converted to power.
type SungrowDC struct {
	implement.Caps
	log      *util.Logger
	conn     *modbus.Connection
	gunBase  uint16 // input register base of the connector's data block
	setBase  uint16 // holding register base of the connector's setpoint block
	rfidBase uint16 // input register base of the connector's household block
	minPower uint32 // W
	maxPower uint32 // W, 0 if unknown
	power    uint32 // W, last setpoint
	enabled  bool
	gunG     func() ([]byte, error)
}

// All register constants hold the wire address. The protocol documents
// addresses that must be accessed minus 1; the documented address is noted.
const (
	// whole charger input registers (FC04)
	sgdcRegSerial      = 21400 // char[20] (doc 21401)
	sgdcRegProtocolId  = 21410 // char[4] "AP00" (doc 21411)
	sgdcRegProtocolVer = 21412 // U32 (doc 21413)
	sgdcRegModel       = 21415 // char[16] (doc 21416)
	sgdcRegDeviceType  = 21423 // U16 (doc 21424)
	sgdcRegSoftwareVer = 21425 // char[30] (doc 21426)
	sgdcRegGunCount    = 21469 // U16 (doc 21470)
	sgdcRegMinPower    = 21471 // U32 W (doc 21472)
	sgdcRegMaxPower    = 21473 // U32 W (doc 21474)
	sgdcRegTotalPower  = 21481 // U32 W (doc 21482)

	// whole charger holding registers (FC03), read only by this driver
	sgdcRegChargerEnable = 21409 // U16 (doc 21410)
	sgdcRegPowerLimit    = 21411 // U32 W (doc 21412)
	sgdcRegOfflinePower  = 21413 // U32 W (doc 21414)
	sgdcRegCommTimeout   = 21415 // U16 s (doc 21416)

	// per-connector input block (FC04), offsets relative to the gun base
	sgdcGunBaseA         = 21499 // gun A (doc 21500), gun B follows 50 registers later
	sgdcGunStride        = 50
	sgdcGunLen           = 43 // through execution power
	sgdcGunTotalEnergy   = 1  // U32 Wh
	sgdcGunSelfDesc      = 3  // U16 bit0: start/stop available, bit1: power setting available
	sgdcGunStartMode     = 4  // U16 0: idle, 1: EMS, 2: RFID
	sgdcGunPowerRequest  = 5  // U16 0: requested, 1: not requested
	sgdcGunPowerAllowed  = 6  // U16 0: off, 1: on
	sgdcGunStatus        = 7  // U16 ChgSts
	sgdcGunDemandVoltage = 12 // U16 0.1V
	sgdcGunDemandCurrent = 13 // U16 0.1A
	sgdcGunOutputVoltage = 14 // U16 0.1V
	sgdcGunOutputCurrent = 15 // U16 0.1A
	sgdcGunPower         = 16 // U32 W
	sgdcGunSoc           = 18 // U16 %
	sgdcGunChargedEnergy = 19 // U32 Wh
	sgdcGunChargeMinutes = 22 // U16 min
	sgdcGunMaxPower      = 23 // U32 W
	sgdcGunMinPower      = 25 // U32 W
	sgdcGunEmsAllowed    = 40 // U16 0: not allowed, 1: allowed
	sgdcGunExecPower     = 41 // U32 W

	// per-connector holding block (FC10), offsets relative to the setpoint base
	sgdcSetBaseA   = 21449 // gun A (doc 21450), gun B follows 30 registers later
	sgdcSetStride  = 30
	sgdcSetPower   = 2 // U32 W output power limit
	sgdcSetCommand = 4 // U32 1: start charging, 2: stop charging

	// per-connector household block (FC04, protocol V1.0.15+), offsets relative to the rfid base
	sgdcRfidBaseA     = 18039 // gun A (doc 18040), gun B follows 60 registers later
	sgdcRfidStride    = 60
	sgdcRfidLen       = 30 // U16 number of card digits
	sgdcRfidCard      = 31 // char[10] packed digits
	sgdcRfidStopCause = 36 // U16 0: charging, >0: stopped

	sgdcCmdStart = 1
	sgdcCmdStop  = 2

	// evcc current setpoints are converted using the 3-phase AC convention
	sgdcPowerPerAmp = 230 * 3 // W/A
)

// charging status (ChgSts)
const (
	sgdcStatusNone          = 0 // undocumented, observed on IDC30E without vehicle
	sgdcStatusIdle          = 1
	sgdcStatusStandby       = 2
	sgdcStatusCharging      = 3
	sgdcStatusPausedCharger = 4
	sgdcStatusPausedVehicle = 5
	sgdcStatusComplete      = 6
	sgdcStatusAppointment   = 7
	sgdcStatusDisabled      = 8
	sgdcStatusFault         = 9
)

var sgdcStatusNames = map[uint16]string{
	sgdcStatusNone:          "None",
	sgdcStatusIdle:          "Idle",
	sgdcStatusStandby:       "Standby",
	sgdcStatusCharging:      "Charging",
	sgdcStatusPausedCharger: "Paused by charger",
	sgdcStatusPausedVehicle: "Paused by vehicle",
	sgdcStatusComplete:      "Complete",
	sgdcStatusAppointment:   "Appointment",
	sgdcStatusDisabled:      "Disabled",
	sgdcStatusFault:         "Fault",
}

var (
	sgdcRegVoltages = []uint16{21460, 21462, 21464} // U16 0.1V (doc 21461/21463/21465)
	sgdcRegCurrents = []uint16{21461, 21463, 21465} // U16 0.1A (doc 21462/21464/21466)
)

func init() {
	registry.AddCtx("sungrow-dc", NewSungrowDCFromConfig)
}

// NewSungrowDCFromConfig creates a Sungrow DC charger from generic config
func NewSungrowDCFromConfig(ctx context.Context, other map[string]any) (api.Charger, error) {
	cc := struct {
		Connector       uint16
		modbus.Settings `mapstructure:",squash"`
	}{
		Connector: 1,
		Settings: modbus.Settings{
			ID: 1,
		},
	}

	if err := util.DecodeOther(other, &cc); err != nil {
		return nil, err
	}

	return NewSungrowDC(ctx, cc.Settings, cc.Connector)
}

// NewSungrowDC creates a Sungrow DC charger
func NewSungrowDC(ctx context.Context, settings modbus.Settings, connector uint16) (api.Charger, error) {
	if connector < 1 || connector > 2 {
		return nil, fmt.Errorf("invalid connector: %d", connector)
	}

	conn, err := settings.Connection(ctx)
	if err != nil {
		return nil, err
	}

	log := util.NewLogger("sungrow-dc")
	conn.Logger(log.TRACE)

	wb := newSungrowDC(log, conn, connector)

	if err := wb.probe(); err != nil {
		return nil, err
	}

	go wb.heartbeat(ctx, wb.heartbeatInterval())

	return wb, nil
}

// newSungrowDC wires the struct without device probes (also used by tests)
func newSungrowDC(log *util.Logger, conn *modbus.Connection, connector uint16) *SungrowDC {
	offset := connector - 1

	wb := &SungrowDC{
		Caps:     implement.New(),
		log:      log,
		conn:     conn,
		gunBase:  sgdcGunBaseA + offset*sgdcGunStride,
		setBase:  sgdcSetBaseA + offset*sgdcSetStride,
		rfidBase: sgdcRfidBaseA + offset*sgdcRfidStride,
	}

	// all cyclic values come from a single cached bulk read of the gun block
	wb.gunG = util.Cached(func() ([]byte, error) {
		return wb.conn.ReadInputRegisters(wb.gunBase, sgdcGunLen)
	}, time.Second)

	return wb
}

// probe verifies the device, determines power limits and registers hardware-dependent capabilities
func (wb *SungrowDC) probe() error {
	b, err := wb.conn.ReadInputRegisters(sgdcRegProtocolId, 2)
	if err != nil {
		return fmt.Errorf("protocol id: %w", err)
	}
	if id := bytesAsString(b); id != "AP00" {
		wb.log.WARN.Printf("unexpected protocol id %q, expected AP00", id)
	}

	gun, err := wb.gunG()
	if err != nil {
		return fmt.Errorf("connector data: %w", err)
	}

	// prefer connector limits, fall back to whole charger limits
	wb.minPower = encoding.Uint32LswFirst(sgdcGun(gun, sgdcGunMinPower, 4))
	wb.maxPower = encoding.Uint32LswFirst(sgdcGun(gun, sgdcGunMaxPower, 4))

	if !sgdcValidU32(wb.minPower) {
		wb.minPower = 0
		if b, err := wb.conn.ReadInputRegisters(sgdcRegMinPower, 2); err == nil && sgdcValidU32(encoding.Uint32LswFirst(b)) {
			wb.minPower = encoding.Uint32LswFirst(b)
		}
	}

	if !sgdcValidU32(wb.maxPower) {
		wb.maxPower = 0
		if b, err := wb.conn.ReadInputRegisters(sgdcRegMaxPower, 2); err == nil && sgdcValidU32(encoding.Uint32LswFirst(b)) {
			wb.maxPower = encoding.Uint32LswFirst(b)
		}
	}

	// seed enabled state so evcc restarts mid-session report the true state
	switch encoding.Uint16(sgdcGun(gun, sgdcGunStatus, 2)) {
	case sgdcStatusCharging, sgdcStatusPausedVehicle:
		wb.enabled = true
	}

	// AC side phase values are whole charger values and not supported by all models
	if wb.phasesSupported() {
		implement.Has(wb, implement.PhaseVoltages(wb.voltages))
		implement.Has(wb, implement.PhaseCurrents(wb.currents))
	}

	// household block only exists from protocol V1.0.15 on
	if b, err := wb.conn.ReadInputRegisters(wb.rfidBase+sgdcRfidLen, 1); err == nil && sgdcValidU16(encoding.Uint16(b)) {
		implement.Has(wb, implement.Identifier(wb.identify))
	}

	return nil
}

// phasesSupported reports whether the AC side phase registers hold real values
func (wb *SungrowDC) phasesSupported() bool {
	for _, reg := range sgdcRegVoltages {
		b, err := wb.conn.ReadInputRegisters(reg, 1)
		if err != nil || !sgdcValidU16(encoding.Uint16(b)) {
			return false
		}
	}

	return true
}

// heartbeatInterval derives the keep-alive period from the charger's EMS communication timeout
func (wb *SungrowDC) heartbeatInterval() time.Duration {
	b, err := wb.conn.ReadHoldingRegisters(sgdcRegCommTimeout, 1)
	if err != nil {
		return 30 * time.Second
	}

	timeout := encoding.Uint16(b)
	if timeout == 0 || !sgdcValidU16(timeout) {
		return 30 * time.Second
	}

	return max(time.Duration(timeout)*time.Second/2, 5*time.Second)
}

// heartbeat keeps the EMS link alive, otherwise the charger falls back to its offline power limit
func (wb *SungrowDC) heartbeat(ctx context.Context, interval time.Duration) {
	for tick := time.Tick(interval); ; {
		select {
		case <-tick:
		case <-ctx.Done():
			return
		}
		if _, err := wb.conn.ReadInputRegisters(wb.gunBase+sgdcGunStatus, 1); err != nil {
			wb.log.ERROR.Println("heartbeat:", err)
		}
	}
}

// sgdcGun returns the bytes of the given register offset within the gun block
func sgdcGun(b []byte, off, n int) []byte {
	return b[2*off : 2*off+n]
}

// sgdcValidU16 reports whether the value is not the "not supported" sentinel
func sgdcValidU16(v uint16) bool {
	return v != math.MaxUint16
}

// sgdcValidU32 reports whether the value is a usable limit, i.e. neither zero nor the "not supported" sentinel
func sgdcValidU32(v uint32) bool {
	return v != 0 && v != math.MaxUint32
}

func (wb *SungrowDC) writeU32(reg uint16, v uint32) error {
	b := make([]byte, 4)
	encoding.PutUint32LswFirst(b, v)

	_, err := wb.conn.WriteMultipleRegisters(reg, 2, b)
	return err
}

// getPhaseValues returns 3 non-sequential register values
func (wb *SungrowDC) getPhaseValues(regs []uint16, divider float64) (float64, float64, float64, error) {
	var res [3]float64
	for i, reg := range regs {
		b, err := wb.conn.ReadInputRegisters(reg, 1)
		if err != nil {
			return 0, 0, 0, err
		}

		res[i] = float64(encoding.Uint16(b)) / divider
	}

	return res[0], res[1], res[2], nil
}

// Status implements the api.Charger interface
func (wb *SungrowDC) Status() (api.ChargeStatus, error) {
	b, err := wb.gunG()
	if err != nil {
		return api.StatusNone, err
	}

	switch s := encoding.Uint16(sgdcGun(b, sgdcGunStatus, 2)); s {
	case sgdcStatusNone, sgdcStatusIdle:
		return api.StatusA, nil
	case sgdcStatusStandby, sgdcStatusAppointment:
		return api.StatusB, nil
	case sgdcStatusCharging:
		wb.enabled = true
		return api.StatusC, nil
	case sgdcStatusPausedCharger, sgdcStatusComplete:
		wb.enabled = false
		return api.StatusB, nil
	case sgdcStatusPausedVehicle:
		wb.enabled = true
		return api.StatusB, nil
	case sgdcStatusDisabled, sgdcStatusFault:
		return api.StatusNone, fmt.Errorf("device state: %s", sgdcStatusNames[s])
	default:
		return api.StatusNone, fmt.Errorf("invalid status: %d", s)
	}
}

// Enabled implements the api.Charger interface
func (wb *SungrowDC) Enabled() (bool, error) {
	return wb.enabled, nil
}

// Enable implements the api.Charger interface
func (wb *SungrowDC) Enable(enable bool) error {
	cmd := uint32(sgdcCmdStop)
	if enable {
		cmd = sgdcCmdStart
	}

	if err := wb.writeU32(wb.setBase+sgdcSetCommand, cmd); err != nil {
		return err
	}

	// re-apply the power limit, the charger may reset it when a session starts
	if enable && wb.power > 0 {
		if err := wb.writeU32(wb.setBase+sgdcSetPower, wb.power); err != nil {
			return err
		}
	}

	wb.enabled = enable

	return nil
}

// MaxCurrent implements the api.Charger interface
func (wb *SungrowDC) MaxCurrent(current int64) error {
	return wb.MaxCurrentMillis(float64(current))
}

var _ api.ChargerEx = (*SungrowDC)(nil)

// MaxCurrentMillis implements the api.ChargerEx interface
func (wb *SungrowDC) MaxCurrentMillis(current float64) error {
	if current <= 0 {
		return fmt.Errorf("invalid current %.3g", current)
	}

	power := max(uint32(current*sgdcPowerPerAmp), wb.minPower)
	if wb.maxPower > 0 {
		power = min(power, wb.maxPower)
	}

	err := wb.writeU32(wb.setBase+sgdcSetPower, power)
	if err == nil {
		wb.power = power
	}

	return err
}

var _ api.PowerLimiter = (*SungrowDC)(nil)

// GetMinMaxPower implements the api.PowerLimiter interface
func (wb *SungrowDC) GetMinMaxPower() (float64, float64, error) {
	if wb.maxPower == 0 {
		return 0, 0, api.ErrNotAvailable
	}

	return float64(wb.minPower), float64(wb.maxPower), nil
}

var _ api.Meter = (*SungrowDC)(nil)

// CurrentPower implements the api.Meter interface
func (wb *SungrowDC) CurrentPower() (float64, error) {
	b, err := wb.gunG()
	if err != nil {
		return 0, err
	}

	return float64(encoding.Uint32LswFirst(sgdcGun(b, sgdcGunPower, 4))), nil
}

var _ api.MeterEnergy = (*SungrowDC)(nil)

// TotalEnergy implements the api.MeterEnergy interface
func (wb *SungrowDC) TotalEnergy() (float64, error) {
	b, err := wb.gunG()
	if err != nil {
		return 0, err
	}

	return float64(encoding.Uint32LswFirst(sgdcGun(b, sgdcGunTotalEnergy, 4))) / 1e3, nil
}

var _ api.ChargeRater = (*SungrowDC)(nil)

// ChargedEnergy implements the api.ChargeRater interface
func (wb *SungrowDC) ChargedEnergy() (float64, error) {
	b, err := wb.gunG()
	if err != nil {
		return 0, err
	}

	return float64(encoding.Uint32LswFirst(sgdcGun(b, sgdcGunChargedEnergy, 4))) / 1e3, nil
}

var _ api.ChargeTimer = (*SungrowDC)(nil)

// ChargeDuration implements the api.ChargeTimer interface
func (wb *SungrowDC) ChargeDuration() (time.Duration, error) {
	b, err := wb.gunG()
	if err != nil {
		return 0, err
	}

	minutes := encoding.Uint16(sgdcGun(b, sgdcGunChargeMinutes, 2))
	if !sgdcValidU16(minutes) {
		return 0, api.ErrNotAvailable
	}

	return time.Duration(minutes) * time.Minute, nil
}

var _ api.Battery = (*SungrowDC)(nil)

// Soc implements the api.Battery interface
func (wb *SungrowDC) Soc() (float64, error) {
	b, err := wb.gunG()
	if err != nil {
		return 0, err
	}

	// zero while no vehicle communicates its soc
	if soc := encoding.Uint16(sgdcGun(b, sgdcGunSoc, 2)); soc > 0 && sgdcValidU16(soc) {
		return float64(soc), nil
	}

	return 0, api.ErrNotAvailable
}

// voltages implements the api.PhaseVoltages interface
func (wb *SungrowDC) voltages() (float64, float64, float64, error) {
	return wb.getPhaseValues(sgdcRegVoltages, 10)
}

// currents implements the api.PhaseCurrents interface
func (wb *SungrowDC) currents() (float64, float64, float64, error) {
	return wb.getPhaseValues(sgdcRegCurrents, 10)
}

// identify implements the api.Identifier interface
func (wb *SungrowDC) identify() ([]string, error) {
	gun, err := wb.gunG()
	if err != nil {
		return nil, err
	}

	// card number is only valid for sessions started by RFID
	if encoding.Uint16(sgdcGun(gun, sgdcGunStartMode, 2)) != 2 {
		return nil, nil
	}

	b, err := wb.conn.ReadInputRegisters(wb.rfidBase+sgdcRfidLen, 6)
	if err != nil {
		return nil, err
	}

	// digits are packed two per byte, e.g. 11223344556677 is reported as 0x11 0x22 ... 0x77
	digits := int(encoding.Uint16(b))
	card := strings.ToUpper(hex.EncodeToString(b[2:]))
	if digits > 0 && digits < len(card) {
		card = card[:digits]
	}

	return []string{card}, nil
}

var _ api.Diagnosis = (*SungrowDC)(nil)

// Diagnose implements the api.Diagnosis interface
func (wb *SungrowDC) Diagnose() {
	if b, err := wb.conn.ReadInputRegisters(sgdcRegSerial, 10); err == nil {
		fmt.Printf("\tSerial:\t%s\n", bytesAsString(b))
	}
	if b, err := wb.conn.ReadInputRegisters(sgdcRegModel, 8); err == nil {
		fmt.Printf("\tModel:\t%s\n", bytesAsString(b))
	}
	if b, err := wb.conn.ReadInputRegisters(sgdcRegDeviceType, 1); err == nil {
		fmt.Printf("\tDevice type:\t0x%04X\n", encoding.Uint16(b))
	}
	if b, err := wb.conn.ReadInputRegisters(sgdcRegProtocolId, 2); err == nil {
		fmt.Printf("\tProtocol:\t%s\n", bytesAsString(b))
	}
	if b, err := wb.conn.ReadInputRegisters(sgdcRegProtocolVer, 2); err == nil {
		fmt.Printf("\tProtocol version:\t%d\n", encoding.Uint32LswFirst(b))
	}
	if b, err := wb.conn.ReadInputRegisters(sgdcRegSoftwareVer, 15); err == nil {
		fmt.Printf("\tSoftware version:\t%s\n", bytesAsString(b))
	}
	if b, err := wb.conn.ReadInputRegisters(sgdcRegGunCount, 1); err == nil {
		fmt.Printf("\tConnectors:\t%d\n", encoding.Uint16(b))
	}
	if b, err := wb.conn.ReadInputRegisters(sgdcRegMinPower, 2); err == nil {
		fmt.Printf("\tCharger min power:\t%d W\n", encoding.Uint32LswFirst(b))
	}
	if b, err := wb.conn.ReadInputRegisters(sgdcRegMaxPower, 2); err == nil {
		fmt.Printf("\tCharger max power:\t%d W\n", encoding.Uint32LswFirst(b))
	}
	if b, err := wb.conn.ReadInputRegisters(sgdcRegTotalPower, 2); err == nil {
		fmt.Printf("\tCharger output power:\t%d W\n", encoding.Uint32LswFirst(b))
	}
	if b, err := wb.conn.ReadHoldingRegisters(sgdcRegChargerEnable, 1); err == nil {
		fmt.Printf("\tCharger enable:\t0x%04X\n", encoding.Uint16(b))
	}
	if b, err := wb.conn.ReadHoldingRegisters(sgdcRegPowerLimit, 2); err == nil {
		fmt.Printf("\tCharger power limit:\t%d W\n", encoding.Uint32LswFirst(b))
	}
	if b, err := wb.conn.ReadHoldingRegisters(sgdcRegOfflinePower, 2); err == nil {
		fmt.Printf("\tOffline power limit:\t%d W\n", encoding.Uint32LswFirst(b))
	}
	if b, err := wb.conn.ReadHoldingRegisters(sgdcRegCommTimeout, 1); err == nil {
		fmt.Printf("\tCommunication timeout:\t%d s\n", encoding.Uint16(b))
	}
	if u1, u2, u3, err := wb.voltages(); err == nil {
		fmt.Printf("\tAC voltages:\t%.1f/%.1f/%.1f V\n", u1, u2, u3)
	}
	if i1, i2, i3, err := wb.currents(); err == nil {
		fmt.Printf("\tAC currents:\t%.1f/%.1f/%.1f A\n", i1, i2, i3)
	}

	b, err := wb.gunG()
	if err != nil {
		return
	}

	status := encoding.Uint16(sgdcGun(b, sgdcGunStatus, 2))
	name := sgdcStatusNames[status]
	if name == "" {
		name = "Unknown"
	}
	fmt.Printf("\tStatus:\t%d (%s)\n", status, name)
	fmt.Printf("\tSelf description:\t0x%04X\n", encoding.Uint16(sgdcGun(b, sgdcGunSelfDesc, 2)))
	fmt.Printf("\tStart mode:\t%d\n", encoding.Uint16(sgdcGun(b, sgdcGunStartMode, 2)))
	fmt.Printf("\tPower request:\t%d\n", encoding.Uint16(sgdcGun(b, sgdcGunPowerRequest, 2)))
	fmt.Printf("\tPower setting allowed:\t%d\n", encoding.Uint16(sgdcGun(b, sgdcGunPowerAllowed, 2)))
	fmt.Printf("\tEMS start allowed:\t%d\n", encoding.Uint16(sgdcGun(b, sgdcGunEmsAllowed, 2)))
	fmt.Printf("\tDemand:\t%.1f V / %.1f A\n", float64(encoding.Uint16(sgdcGun(b, sgdcGunDemandVoltage, 2)))/10, float64(encoding.Uint16(sgdcGun(b, sgdcGunDemandCurrent, 2)))/10)
	fmt.Printf("\tOutput:\t%.1f V / %.1f A\n", float64(encoding.Uint16(sgdcGun(b, sgdcGunOutputVoltage, 2)))/10, float64(encoding.Uint16(sgdcGun(b, sgdcGunOutputCurrent, 2)))/10)
	fmt.Printf("\tConnector min power:\t%d W\n", encoding.Uint32LswFirst(sgdcGun(b, sgdcGunMinPower, 4)))
	fmt.Printf("\tConnector max power:\t%d W\n", encoding.Uint32LswFirst(sgdcGun(b, sgdcGunMaxPower, 4)))
	fmt.Printf("\tExecution power:\t%d W\n", encoding.Uint32LswFirst(sgdcGun(b, sgdcGunExecPower, 4)))

	if b, err := wb.conn.ReadInputRegisters(wb.rfidBase+sgdcRfidStopCause, 1); err == nil {
		fmt.Printf("\tStop reason:\t0x%04X\n", encoding.Uint16(b))
	}
}
