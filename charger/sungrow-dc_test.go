package charger

import (
	"context"
	"net"
	"sync"
	"testing"
	"time"

	"github.com/andig/mbserver"
	"github.com/evcc-io/evcc/api"
	"github.com/evcc-io/evcc/util"
	"github.com/evcc-io/evcc/util/modbus"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

type sgdcWrite struct {
	funcCode uint8
	addr     uint16
	args     []uint16
}

// sgdcHandler mocks the charger's input and holding register space
type sgdcHandler struct {
	mbserver.RequestHandler
	input      map[uint16]uint16
	holding    map[uint16]uint16
	writes     []sgdcWrite
	failWrites bool
}

func (h *sgdcHandler) HandleInputRegisters(req *mbserver.InputRegistersRequest) ([]uint16, error) {
	res := make([]uint16, 0, req.Quantity)
	for i := range req.Quantity {
		v, ok := h.input[req.Addr+i]
		if !ok {
			return nil, mbserver.ErrIllegalDataAddress
		}
		res = append(res, v)
	}
	return res, nil
}

func (h *sgdcHandler) HandleHoldingRegisters(req *mbserver.HoldingRegistersRequest) ([]uint16, error) {
	if req.IsWrite {
		if h.failWrites {
			return nil, mbserver.ErrIllegalDataValue
		}
		h.writes = append(h.writes, sgdcWrite{req.WriteFuncCode, req.Addr, req.Args})
		return req.Args, nil
	}

	res := make([]uint16, 0, req.Quantity)
	for i := range req.Quantity {
		v, ok := h.holding[req.Addr+i]
		if !ok {
			return nil, mbserver.ErrIllegalDataAddress
		}
		res = append(res, v)
	}
	return res, nil
}

// sgdcPutU32 stores a low-word-first U32 at addr
func sgdcPutU32(regs map[uint16]uint16, addr uint16, v uint32) {
	regs[addr] = uint16(v)
	regs[addr+1] = uint16(v >> 16)
}

// sgdcGunBase returns the input register base of the given connector
func sgdcGunBase(connector uint16) uint16 {
	return sgdcGunBaseA + (connector-1)*sgdcGunStride
}

// sgdcRegs returns a zeroed gun block for the connector with the given status plus the protocol id
func sgdcRegs(connector, status uint16) map[uint16]uint16 {
	regs := make(map[uint16]uint16)

	base := sgdcGunBase(connector)
	for reg := base; reg < base+sgdcGunLen; reg++ {
		regs[reg] = 0
	}
	regs[base+sgdcGunStatus] = status

	regs[sgdcRegProtocolId] = 0x4150   // "AP"
	regs[sgdcRegProtocolId+1] = 0x3030 // "00"

	return regs
}

// shared mock server: mbserver.Stop() races its accept goroutine, so the
// server is started once and never stopped; handler state is reset per test
var (
	sgdcOnce sync.Once
	sgdcURI  string
	sgdcSrvH = &sgdcHandler{RequestHandler: new(mbserver.DummyHandler)}
)

// sgdcTestCharger connects a charger for the connector to the shared mock Modbus server without probing
func sgdcTestCharger(t *testing.T, connector uint16, input map[uint16]uint16) (*SungrowDC, *sgdcHandler) {
	t.Helper()

	sgdcOnce.Do(func() {
		l, err := net.Listen("tcp", "localhost:0")
		require.NoError(t, err)

		srv, err := mbserver.New(sgdcSrvH)
		require.NoError(t, err)
		require.NoError(t, srv.Start(l))

		sgdcURI = l.Addr().String()
	})

	sgdcSrvH.input = input
	sgdcSrvH.holding = map[uint16]uint16{}
	sgdcSrvH.writes = nil
	sgdcSrvH.failWrites = false

	conn, err := modbus.NewConnection(context.Background(), sgdcURI, "", "", 0, modbus.Tcp, 249)
	require.NoError(t, err)

	wb := newSungrowDC(util.NewLogger("test"), conn, connector)
	wb.minPower = 2000
	wb.maxPower = 30000

	return wb, sgdcSrvH
}

func TestSungrowDCStatus(t *testing.T) {
	tc := []struct {
		state  uint16
		status api.ChargeStatus
		err    bool
	}{
		{sgdcStatusNone, api.StatusA, false},
		{sgdcStatusIdle, api.StatusA, false},
		{sgdcStatusStandby, api.StatusB, false},
		{sgdcStatusCharging, api.StatusC, false},
		{sgdcStatusPausedCharger, api.StatusB, false},
		{sgdcStatusPausedVehicle, api.StatusB, false},
		{sgdcStatusComplete, api.StatusB, false},
		{sgdcStatusAppointment, api.StatusB, false},
		{sgdcStatusDisabled, api.StatusNone, true},
		{sgdcStatusFault, api.StatusNone, true},
		{10, api.StatusNone, true},
	}

	for _, tc := range tc {
		wb, _ := sgdcTestCharger(t, 1, sgdcRegs(1, tc.state))

		status, err := wb.Status()
		if tc.err {
			assert.Error(t, err, "state %d", tc.state)
		} else {
			assert.NoError(t, err, "state %d", tc.state)
		}
		assert.Equal(t, tc.status, status, "state %d", tc.state)
	}
}

func TestSungrowDCEnabledSync(t *testing.T) {
	tc := []struct {
		state   uint16
		before  bool
		enabled bool
	}{
		{sgdcStatusCharging, false, true},
		{sgdcStatusPausedVehicle, false, true},
		{sgdcStatusPausedCharger, true, false},
		{sgdcStatusComplete, true, false},
		{sgdcStatusStandby, true, true}, // ambiguous state leaves the cache untouched
		{sgdcStatusIdle, false, false},
	}

	for _, tc := range tc {
		wb, _ := sgdcTestCharger(t, 1, sgdcRegs(1, tc.state))
		wb.enabled = tc.before

		_, err := wb.Status()
		require.NoError(t, err, "state %d", tc.state)

		enabled, err := wb.Enabled()
		require.NoError(t, err)
		assert.Equal(t, tc.enabled, enabled, "state %d", tc.state)
	}
}

func TestSungrowDCMeasurements(t *testing.T) {
	regs := sgdcRegs(1, sgdcStatusCharging)
	base := sgdcGunBase(1)
	sgdcPutU32(regs, base+sgdcGunPower, 11500)         // 11500 W
	sgdcPutU32(regs, base+sgdcGunTotalEnergy, 100000)  // 100.000 kWh, spans both words
	sgdcPutU32(regs, base+sgdcGunChargedEnergy, 12340) // 12.34 kWh
	regs[base+sgdcGunChargeMinutes] = 90
	regs[base+sgdcGunSoc] = 65

	wb, _ := sgdcTestCharger(t, 1, regs)

	power, err := wb.CurrentPower()
	require.NoError(t, err)
	assert.Equal(t, 11500.0, power)

	total, err := wb.TotalEnergy()
	require.NoError(t, err)
	assert.Equal(t, 100.0, total)

	charged, err := wb.ChargedEnergy()
	require.NoError(t, err)
	assert.Equal(t, 12.34, charged)

	dur, err := wb.ChargeDuration()
	require.NoError(t, err)
	assert.Equal(t, 90*time.Minute, dur)

	soc, err := wb.Soc()
	require.NoError(t, err)
	assert.Equal(t, 65.0, soc)
}

func TestSungrowDCNotAvailable(t *testing.T) {
	// zero soc means no vehicle data
	wb, _ := sgdcTestCharger(t, 1, sgdcRegs(1, sgdcStatusIdle))
	_, err := wb.Soc()
	assert.ErrorIs(t, err, api.ErrNotAvailable)

	// unsupported registers answer the sentinel
	regs := sgdcRegs(1, sgdcStatusIdle)
	regs[sgdcGunBase(1)+sgdcGunSoc] = 0xFFFF
	regs[sgdcGunBase(1)+sgdcGunChargeMinutes] = 0xFFFF

	wb, _ = sgdcTestCharger(t, 1, regs)
	_, err = wb.Soc()
	assert.ErrorIs(t, err, api.ErrNotAvailable)
	_, err = wb.ChargeDuration()
	assert.ErrorIs(t, err, api.ErrNotAvailable)
}

func TestSungrowDCEnable(t *testing.T) {
	wb, h := sgdcTestCharger(t, 1, sgdcRegs(1, sgdcStatusStandby))
	wb.power = 12000

	require.NoError(t, wb.Enable(true))
	enabled, err := wb.Enabled()
	require.NoError(t, err)
	assert.True(t, enabled)

	require.NoError(t, wb.Enable(false))
	enabled, err = wb.Enabled()
	require.NoError(t, err)
	assert.False(t, enabled)

	// start re-applies the power limit, stop only sends the command; all U32 low word first
	require.Len(t, h.writes, 3)
	assert.Equal(t, sgdcWrite{16, sgdcSetBaseA + sgdcSetCommand, []uint16{sgdcCmdStart, 0}}, h.writes[0])
	assert.Equal(t, sgdcWrite{16, sgdcSetBaseA + sgdcSetPower, []uint16{12000, 0}}, h.writes[1])
	assert.Equal(t, sgdcWrite{16, sgdcSetBaseA + sgdcSetCommand, []uint16{sgdcCmdStop, 0}}, h.writes[2])
}

func TestSungrowDCEnableWithoutSetpoint(t *testing.T) {
	wb, h := sgdcTestCharger(t, 1, sgdcRegs(1, sgdcStatusStandby))

	require.NoError(t, wb.Enable(true))

	// no power limit known yet, never write 0 W
	require.Len(t, h.writes, 1)
	assert.Equal(t, sgdcWrite{16, sgdcSetBaseA + sgdcSetCommand, []uint16{sgdcCmdStart, 0}}, h.writes[0])
}

func TestSungrowDCEnableWriteFailure(t *testing.T) {
	wb, h := sgdcTestCharger(t, 1, sgdcRegs(1, sgdcStatusStandby))
	h.failWrites = true

	assert.Error(t, wb.Enable(true))

	// failed write must not flip the cached state
	enabled, err := wb.Enabled()
	require.NoError(t, err)
	assert.False(t, enabled)
}

func TestSungrowDCMaxCurrent(t *testing.T) {
	tc := []struct {
		current float64
		power   uint32
	}{
		{6, 4140},
		{16, 11040},
		{40, 27600},
		{50, 30000}, // 34500 W clamped to max power
		{1, 2000},   // 690 W raised to min power
	}

	for _, tc := range tc {
		wb, h := sgdcTestCharger(t, 1, sgdcRegs(1, sgdcStatusCharging))

		require.NoError(t, wb.MaxCurrentMillis(tc.current))
		require.Len(t, h.writes, 1, "current %v", tc.current)

		assert.Equal(t, sgdcWrite{16, sgdcSetBaseA + sgdcSetPower, []uint16{uint16(tc.power), uint16(tc.power >> 16)}}, h.writes[0], "current %v", tc.current)
		assert.Equal(t, tc.power, wb.power, "current %v", tc.current)
	}
}

func TestSungrowDCMaxCurrentInvalid(t *testing.T) {
	wb, h := sgdcTestCharger(t, 1, sgdcRegs(1, sgdcStatusCharging))

	assert.Error(t, wb.MaxCurrentMillis(0))
	assert.Error(t, wb.MaxCurrentMillis(-1))
	assert.Empty(t, h.writes)
}

func TestSungrowDCMinMaxPower(t *testing.T) {
	wb, _ := sgdcTestCharger(t, 1, sgdcRegs(1, sgdcStatusIdle))

	minP, maxP, err := wb.GetMinMaxPower()
	require.NoError(t, err)
	assert.Equal(t, 2000.0, minP)
	assert.Equal(t, 30000.0, maxP)

	wb.maxPower = 0
	_, _, err = wb.GetMinMaxPower()
	assert.ErrorIs(t, err, api.ErrNotAvailable)
}

func TestSungrowDCProbePowerLimits(t *testing.T) {
	base := sgdcGunBase(1)

	// connector limits take precedence
	regs := sgdcRegs(1, sgdcStatusIdle)
	sgdcPutU32(regs, base+sgdcGunMinPower, 1500)
	sgdcPutU32(regs, base+sgdcGunMaxPower, 22000)
	sgdcPutU32(regs, sgdcRegMinPower, 2000)
	sgdcPutU32(regs, sgdcRegMaxPower, 30000)

	wb, _ := sgdcTestCharger(t, 1, regs)
	require.NoError(t, wb.probe())
	assert.Equal(t, uint32(1500), wb.minPower)
	assert.Equal(t, uint32(22000), wb.maxPower)

	// unsupported connector limits fall back to the whole charger
	regs = sgdcRegs(1, sgdcStatusIdle)
	sgdcPutU32(regs, base+sgdcGunMinPower, 0xFFFFFFFF)
	sgdcPutU32(regs, base+sgdcGunMaxPower, 0xFFFFFFFF)
	sgdcPutU32(regs, sgdcRegMinPower, 2000)
	sgdcPutU32(regs, sgdcRegMaxPower, 30000)

	wb, _ = sgdcTestCharger(t, 1, regs)
	require.NoError(t, wb.probe())
	assert.Equal(t, uint32(2000), wb.minPower)
	assert.Equal(t, uint32(30000), wb.maxPower)

	// no limits at all
	wb, _ = sgdcTestCharger(t, 1, sgdcRegs(1, sgdcStatusIdle))
	require.NoError(t, wb.probe())
	assert.Equal(t, uint32(0), wb.minPower)
	assert.Equal(t, uint32(0), wb.maxPower)
	_, _, err := wb.GetMinMaxPower()
	assert.ErrorIs(t, err, api.ErrNotAvailable)
}

func TestSungrowDCProbeSeedsEnabled(t *testing.T) {
	wb, _ := sgdcTestCharger(t, 1, sgdcRegs(1, sgdcStatusCharging))
	require.NoError(t, wb.probe())
	enabled, _ := wb.Enabled()
	assert.True(t, enabled)

	wb, _ = sgdcTestCharger(t, 1, sgdcRegs(1, sgdcStatusStandby))
	require.NoError(t, wb.probe())
	enabled, _ = wb.Enabled()
	assert.False(t, enabled)
}

func TestSungrowDCProbeFailsWithoutProtocolId(t *testing.T) {
	regs := sgdcRegs(1, sgdcStatusIdle)
	delete(regs, uint16(sgdcRegProtocolId))

	wb, _ := sgdcTestCharger(t, 1, regs)
	assert.Error(t, wb.probe())
}

func TestSungrowDCProbePhases(t *testing.T) {
	// sentinel values: no phase capabilities
	regs := sgdcRegs(1, sgdcStatusIdle)
	for _, reg := range sgdcRegVoltages {
		regs[reg] = 0xFFFF
	}
	for _, reg := range sgdcRegCurrents {
		regs[reg] = 0xFFFF
	}

	wb, _ := sgdcTestCharger(t, 1, regs)
	require.NoError(t, wb.probe())
	assert.False(t, api.HasCap[api.PhaseVoltages](wb))
	assert.False(t, api.HasCap[api.PhaseCurrents](wb))

	// real values: capabilities registered and readable
	for i, reg := range sgdcRegVoltages {
		regs[reg] = uint16(2300 + i)
	}
	for i, reg := range sgdcRegCurrents {
		regs[reg] = uint16(100 + i)
	}

	wb, _ = sgdcTestCharger(t, 1, regs)
	require.NoError(t, wb.probe())

	pv, ok := api.Cap[api.PhaseVoltages](wb)
	require.True(t, ok)
	u1, u2, u3, err := pv.Voltages()
	require.NoError(t, err)
	assert.Equal(t, []float64{230, 230.1, 230.2}, []float64{u1, u2, u3})

	pc, ok := api.Cap[api.PhaseCurrents](wb)
	require.True(t, ok)
	i1, i2, i3, err := pc.Currents()
	require.NoError(t, err)
	assert.Equal(t, []float64{10, 10.1, 10.2}, []float64{i1, i2, i3})
}

func TestSungrowDCProbeIdentifier(t *testing.T) {
	// household block missing: no identifier
	wb, _ := sgdcTestCharger(t, 1, sgdcRegs(1, sgdcStatusIdle))
	require.NoError(t, wb.probe())
	assert.False(t, api.HasCap[api.Identifier](wb))

	// household block present
	regs := sgdcRegs(1, sgdcStatusCharging)
	regs[sgdcGunBase(1)+sgdcGunStartMode] = 2 // RFID start
	regs[sgdcRfidBaseA+sgdcRfidLen] = 14
	regs[sgdcRfidBaseA+sgdcRfidCard] = 0x1122
	regs[sgdcRfidBaseA+sgdcRfidCard+1] = 0x3344
	regs[sgdcRfidBaseA+sgdcRfidCard+2] = 0x5566
	regs[sgdcRfidBaseA+sgdcRfidCard+3] = 0x7700
	regs[sgdcRfidBaseA+sgdcRfidCard+4] = 0x0000

	wb, _ = sgdcTestCharger(t, 1, regs)
	require.NoError(t, wb.probe())

	id, ok := api.Cap[api.Identifier](wb)
	require.True(t, ok)
	ids, err := id.Identify()
	require.NoError(t, err)
	assert.Equal(t, []string{"11223344556677"}, ids)

	// sessions not started by card carry no id
	regs[sgdcGunBase(1)+sgdcGunStartMode] = 1
	wb, _ = sgdcTestCharger(t, 1, regs)
	require.NoError(t, wb.probe())
	id, _ = api.Cap[api.Identifier](wb)
	ids, err = id.Identify()
	require.NoError(t, err)
	assert.Empty(t, ids)
}

func TestSungrowDCConnectorOffsets(t *testing.T) {
	regs := sgdcRegs(2, sgdcStatusCharging)
	sgdcPutU32(regs, sgdcGunBase(2)+sgdcGunPower, 7000)

	wb, h := sgdcTestCharger(t, 2, regs)

	status, err := wb.Status()
	require.NoError(t, err)
	assert.Equal(t, api.StatusC, status)

	power, err := wb.CurrentPower()
	require.NoError(t, err)
	assert.Equal(t, 7000.0, power)

	require.NoError(t, wb.MaxCurrentMillis(10))
	require.NoError(t, wb.Enable(false))
	require.Len(t, h.writes, 2)
	assert.Equal(t, uint16(sgdcSetBaseA+sgdcSetStride+sgdcSetPower), h.writes[0].addr)
	assert.Equal(t, uint16(sgdcSetBaseA+sgdcSetStride+sgdcSetCommand), h.writes[1].addr)
}

func TestSungrowDCInvalidConnector(t *testing.T) {
	_, err := NewSungrowDC(t.Context(), modbus.Settings{URI: "localhost:0"}, 3)
	assert.Error(t, err)
}

func TestSungrowDCHeartbeatInterval(t *testing.T) {
	wb, h := sgdcTestCharger(t, 1, sgdcRegs(1, sgdcStatusIdle))

	// no timeout register: default
	assert.Equal(t, 30*time.Second, wb.heartbeatInterval())

	// half of the device timeout
	h.holding[sgdcRegCommTimeout] = 120
	assert.Equal(t, 60*time.Second, wb.heartbeatInterval())

	// lower bound
	h.holding[sgdcRegCommTimeout] = 4
	assert.Equal(t, 5*time.Second, wb.heartbeatInterval())
}
