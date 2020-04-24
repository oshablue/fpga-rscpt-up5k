//
// Implementation:
// 80MHz output PLL from 16MHz oscillator input (CLK)
// Sampling at 40MHz with divide by 2 sample clock
// and the use of sync for all async signals ends up dividing by 2
// So you see some code like monostable vibrator for example running
// with twice the counter length
//
//
// Status:
// vbak.good02: Looks functional, async issues addressed - continuous
// sweep input signals look intact at the read output from arduino test code
// and capturing indeed the 4096 samples at 40MHz (have tested eg 400 samples
// of a 100kHz waveform at 40MHz and indeed this shows 1 full cycle on the
// plotter - indicating correct sampling in time and sample storage)
//
// Issue remaining:
// - is the always vertical, sometimes occurring spike on the eg 100kHz wf
//   due to timing/memory/async issue or just actual noise/bit issue in the
//   prototype setup

// Notes:
// All Lattice devices have weak pull up resistors on IO


// For turning on the selfread so that after capture, data is dumped in parallel
// out to some external fifo for example
// Tested - so far yes still works as well with manually clocked out data from
// nano program arduino with this macro turned off
`define SELFREAD 1

// For use with nano arduino clocked read sample output and also read into
// the external fifo (write into it really):
//`define SYNC_SELFREAD_WITH_CLOCKED_OUT 1

// For external fifo - note that documentation for the FT2232 chip shows different
// implementations in different areas.  However, currently for the FT2232H
// datasheet, the wr# is, as it is named with the "#", active low, thus idle high
// Table 3.7 in DS_FT2232H
// See Figure 4.6 in DS_FT2232H - this matches idle high, active low strobe
// with the write-into-2232-fifo happening on the falling edge
// As long as TXE# is not high
`define EXT_FIFO_WR_ACTIVE  1'b 0
`define EXT_FIFO_WR_IDLE    1'b 1


// Here's the deal with PLL implementation
// Ok - here are the refs:
// Err... see the Readme
// https://github.com/mystorm-org/BlackIce-II/wiki/PLLs-Advanced
// https://github.com/YosysHQ/arachne-pnr/issues/64
// You see my friend, apparently, when the PLLSOURCE pin (external)
// is IOB bottom Bank 1 (for example we were/are using in our design G3/IOB_25b)
// this compiles fine using SB_PLL40_CORE
// HOWEVER
// When CLK input goes to eg Pin 35 which is IOT_46B_G0 (Top, Bank 0)
// the build breaks and arachne-pnr can't place the PLL so we need to use
// PLL 2 which allows both an external pin to be used both to drive the PLL
// and internally on the FPGA (as an output from the PLL block)
// Defining the item below is like setting CLK (in from external oscillator)
// to Pin 35 like for use with the Lattice Eval Board (UltraPlus Breakout Board)
// Whereas undefining it is like using our planned implementation and setting the
// CLK input pin to be eg Pin 20
// Using Pin defs that don't match the imlementation and not using the correct
// PLL (for example allowing the item below to be defined, but keeping CLK on
// pin 20) will break the build.
//
// Uncomment to use breakout board with Pin 35 as the clock input (external)
// Comment it to use our pin 20 implementation in the hardware HDL-0108-RSCPT
// COMMENT OUT NEXT define LINE FOR HDL HARDWARE
// UNCOMMENT NEXT define LINE FOR LATTICE SEMI DEMO BOARD
//`define USE_LATTICE_BREAKOUT_DEMO 1'b0
// Thus below, not defined for HDL-0108-RSCPT config/hw
`ifdef USE_LATTICE_BREAKOUT_DEMO
  `define PLLSOURCE_BOTTOM_BANK  1'b0
`endif
// CAUTION: Have not yet tested with the above commented out and then swapping
// PCF file for the target build
// THOUGH: Tested with not defined USE_LATTICE_BREAKOUT_DEMO and using
// pins-up5k-sg48.pcf for pins as implemented on the HDL-0108-RSCPT R01 A1


// Choose between using the read_mem_in implementation or otherwise
// using the read_mem_in line instead as rx-delay-bit selection
// Could also otherwise implement the rx-delay as a single line with external
// control whose state simply defines the rx-delay after capture init
// Use either both of the top two, or just the 3rd item
// This implementation could obviously be cleaned up - just looking to retain
// functionality during development.
// These also relate to the SELFREAD - so please coordinate the defines.
// They are more granular here for dev reasons.
//`define USE_READ_MEM_IN_AS_READ_MEM_IN
//`define USE_CAPT_DONE_AS_CAPT_DONE
`define USE_RX_DELAY_CTRL_2BIT


// TODO - noticed some potential noise at 0 Rx delay with updated
// multi rx delay implemented - esp. at zero points
// Tested various combos of reverting back to no rx delay and
// moving capture (from data_in) timing to be based on posedge sclk for example
// and all effect jitter or adc settling noise, it seems.
// For now, implementation is ok - however possible improvement by
// creating a delayed clock for the data_in grab that is sync'd to a delay
// from the capture signal by some small amount
// Thus perhaps running internally at 160MHz if possible (or similar)
// And using the same SAMPLECLK (80MHz) for the main timing functions,
// the divided by 2 clock (sclk) for the adc, and then uncomment the
// block below, swapping it, for a slightly delayed grab from data_in such
// that it is basically the only thing running at a slightly delayed clock edge,
// yet still within the timing needed before next capture and data shifts, etc.

// Confirmations that indeed we are running at 40MHz sample capture (encode)
// clock and that the returned samples are correct in time, showing the
// correct waveform in time, eg thickness as calculated from the return
// in real life is correct


// UART output for e.g. RS-485/422 style conversion output drive in our particular
// system
//`define USE_UART 1



// For TESTING on the LAttice Demo Board
// Using leds
//`define FLASH_LEDS 1



// TODO - Implemented TXE# (fifo_txe)

// For divide by N
`define CLOG2(x) \
   x <= 2	 ? 1 : \
   x <= 4	 ? 2 : \
   x <= 8	 ? 3 : \
   x <= 16	 ? 4 : \
   x <= 32	 ? 5 : \
   x <= 64	 ? 6 : \
   x <= 128	 ? 7 : \
   x <= 256	 ? 8 : \
   x <= 512	 ? 9 : \
   x <= 1024	 ? 10 : \
   x <= 2048	 ? 11 : \
   x <= 4096	 ? 12 : \
   x <= 8192	 ? 13 : \
   x <= 16384	 ? 14 : \
   x <= 32768	 ? 15 : \
   x <= 65536	 ? 16 : \
   -1



module top (
    input CLK,                  // 16MHz clock
    input RST,                  // RST for PLL?
    input trig_in,              // trigger a capture event sequence from comms device
    //input read_mem_in,             // read strobe from comms device
    // repurpose read_mem_in as input for rx_delay_ctrl_b0:
    //input rx_delay_ctrl_b0,
    //input rx_delay_ctrl_b1,
    input [1:0] rx_delay_ctrl,
    input [7:0] data_in,
    input fifo_txe,             // FIFO ok to write (buffer empty/write to FIFO enabled)
    //output LED,                 // User/boot LED next to power LED
    //output USBPU,               // USB pull-up resistor
    //output SAMPLECLK,           // capture output from PLL
    //output SAMPLECLK8X,         //
    //output clk_lock,
    output [7:0] data_out,      // testing external LED
    // re-purpose capt_done from output to input for rx_delay_ctrl_b1
    //output capt_done,           // capture event is complete
    output adc_encode,           // encode signal to ADC to capture a sample
    output fifo_wri,             // FIFO write strobe
    output PULSE_NEG,
    output RTZ_NEG,
    output RTZ_POS

  `ifdef USE_LATTICE_BREAKOUT_DEMO  // Not true for HDL-0108-RSCPT config/hw
    ,
    output RGB0,                 // status output
    output RGB1,
    output RGB2                   // last LED status indic
  `endif

  `ifdef USE_UART
    ,
    output UART_TX,
    input UART_RX
  `endif
);



    parameter NSAMPLES = 12'b 1111_1111_1111; // 4095

    // When not using external pins:
    wire SAMPLECLK, SAMPLECLK8X, clk_lock, clk160;

    wire read_mem;

    wire clk_copy;

    reg[7:0] seq_id;  // sequence ID for the frame
    reg[7:0] seq_id_nxt;



  `ifndef PLLSOURCE_BOTTOM_BANK // We are here for HDL-0108-RSCPT config/hw
    pllcore p(
        .clk    (CLK),          // 16MHz oscillator input
        .clkin  (SAMPLECLK8X),  // now a non-pin
        .clkout (clk160), //SAMPLECLK),    // now a non-pin
        .lock   (clk_lock)      // now a non-pin
    );
    assign clk_copy = CLK;
  `else
    pll2pad p(
        .clk_in   (CLK),        // Input that is a pad/pin (real) //
        .clkout_a (clk_copy),   // Output that is a copy of the pin in clk
        .clkout_b (clk160),     // Output that is PLL out // was: SAMPLECLK
        .lock     (clk_lock)    // Pin 37 is no longer available as RST (input) when using this PLL version -- see PLL modules
    );
  `endif


    // Not current:
    // For 40MHz clock out from PLL:
    //assign adc_encode = SAMPLECLK;




    // clk160: 160 MHz
    // SAMPLECLK = 80 MHz
    // sclk = 40 MHz
    wire sclk;
    divide_by_n #(.N(2)) div160to80(clk160, 1'b0, SAMPLECLK);
    //divide_by_n #(.N(2)) divsclk(SAMPLECLK, 1'b0, sclk);
    divide_by_n #(.N(4)) divsclk(clk160, 1'b0, sclk);
    assign adc_encode = (sclk);







    // Not for HDL-0108-RSCPT config/hw
  `ifdef USE_LATTICE_BREAKOUT_DEMO        // Not for HDL-0108-RSCPT config/hw
  `ifdef FLASH_LEDS
    // Status LED output (?) ... testing
    wire pretty_slow;
    wire clk_super_slow;
    reg [2:0] ledsreg = 3'b 110;  // LEDs OFF = 1 ... ON = 0
    reg [2:0] leds = 3'b 000;
    reg [2:0] ledson = 3'b 111;
    assign RGB0 = leds[2];  // B
    assign RGB1 = leds[1];  // G
    assign RGB2 = leds[0];  // R
    // instead of RST try: 1'b0
    // yeah if we use RST -- this pin is currently confusing things
    // so for now, just use 1'b0 for the reset instead of the RST
    //divide_by_n #(.N(256)) div_slow(CLK, 1'b0, clk_slow);
    // Yeah testing with 256 is 16MHz / 256 = 62.5 kHz and yes this is the output
    divide_by_n #(.N(65536)) div_pretty_slow(clk_copy, 1'b0, pretty_slow);
    divide_by_n #(.N(32)) div_super_slow(pretty_slow, 1'b0, clk_super_slow);
    // TODO FOR FUN: Change to PWM and Rate MOD
    always @ ( posedge clk_super_slow) begin
      // Or could do with modulus and counter, etc.
      ledsreg <= {ledsreg[1:0], ledsreg[2]};  // left rotate with carry
      leds <= (ledsreg | ledson);             // | not & because 0 = On
      ledson <= ~ledson;                      // alt on / off
    end
  `else // if not define FLASH_LEDS
    // 1'b1 = OFF ... 1'b0 = ON
    // RGB0 = blue
    //assign RGB0 = 1'b1;
    //assign RGB1 = 1'b0; // at the moment, RGB0 flashing only works when RGB1 is on too ... it seems
    //assign RGB2 = 1'b1;
  `endif
  `endif



  `ifdef SELFREAD
    // TODO this output clock is very dirty ... or is it some IO interaction w/ FIFO external?
    // Yes, below, using 32 divider with bit toggle creates
    // a 50% duty clock at about 1.25MHz (complete cycle is ie about 800ns)
    // So this gives plenty of setup time on the WR# signal for the ext fifo
    // TODO the whole duration of the clock pulses is currently 204us which is
    // only 255 samples (hey, at least it's a multiple of 2 ;)
    // So ... next ...
    // Now duration of all write pulses is 3.30 ms
    // and still at 1.25MHz effective clock rate (800ns)
    // So this is: 4125 or yeah, looks like we're basically getting our
    // 4096 sample dump now
    // TODO why are we getting those voltage-truncate pulses now when the
    // 2232 is powered on and running and enabled -- voltage goes from the first
    // few pulses at 3.3V to somewhere around 2.5? volts - number of pulses varies
    // TXE signal is always high
    // TODO unless it's due to the nano always reading the pins too?
    wire dumpclkedges;
    // 12Mbps => FIFO 8-bit at 1.5MB/s (approx)
    // 80 MHz / 1.5 => 53 => 64 divider is the closest
    // 80 MHz / 64 => 1.25MHz
    // However, the dividy_by_n actually is only spikes (mini pulses at the divide interval)
    // So we create a /64 clock by using 32 and then using the toggle below
    // Below in most testing functional using 32
    // To test frequency clock noise issues, slowing this down a lot
    divide_by_n #(.N(32)) divdumpclk(SAMPLECLK, 1'b0, dumpclkedges);
    reg dumpclkreg;
    wire dumpclk;
    assign dumpclk = dumpclkreg;
    always @( posedge SAMPLECLK )
    begin
      if ( trigd | !new_capt ) begin
        dumpclkreg <= `EXT_FIFO_WR_IDLE; //1'b 0;
      end
      if ( dumpclkedges & new_capt ) begin
        dumpclkreg <= ~dumpclkreg;
      end
    end
  `endif



    reg [11:0] addr_wr, addr_wr_nxt, addr_rd, addr_rd_nxt;
    reg [7:0] val;
    reg [7:0] mem [0:4095];    // Check hardware.out :: yeah, this gens the SB_RAM40_4K blocks





    // Regarding initial values and their non-use:
    // https://github.com/YosysHQ/yosys/issues/103



    assign data_out = val;
  `ifdef USE_CAPT_DONE_AS_CAPT_DONE // currently we're not in here for HDL-0108-RSCPT first fw deployed
    assign capt_done = !trigd;
  `endif









    //
    //
    // Pulse ctrl
    //
    //

    wire
      pulse_on, delay_to_rtz, pulse_rtz;

    wire delay_to_capture;        // Rx (receive) signal capture delay

    assign PULSE_NEG = pulse_on;
    assign RTZ_NEG = pulse_rtz;
    assign RTZ_POS = pulse_rtz;

    wire trig_in_rise;  // This signal comes from the edge detect module on trig_in, the actual external input signal


//`define DEMO_VAR_FREQ_PULSE



`ifndef DEMO_VAR_FREQ_PULSE
    //assign pulse_on = pulse_on1;
    //assign pulse_rtz = pulse_rtz1;


    // TODO we can definitely parameterize the VAR_FREQ / swept frequency

    //
    //
    //
    // First pulse

    // Yes scope-verified that with of 12 gives a pulse width that measures as 7MHz
    // 24 width => -140VDC on 3.5MHz Comp under test
    // Slope limit of the voltage with time likely due to damping in system and ohmage
    // versus with 12/12 pulse width and rtz delay time, voltage to -100VDC or so in neg pulse
    // and
    monostable #(
      .PULSE_WIDTH(12),           // 80MHz / 7MHz (for 2x3.5MHz for half-cycle on hold)
      .COUNTER_WIDTH(5)           // 5 wires for up to count of 32
                                  // Measured (ctrl sig?) pulse width on scope is about 152 ns
    ) msv_pulse_on (
      .clk        (SAMPLECLK),
      .reset      (trig_in_rise),
      .trigger    (trig_in_rise),
      .pulse      (pulse_on)
    );


    // Width of 1: Max neg voltage is less for the 3.5MHz XD under test
    // Width of 3: Slightly more negative voltage
    // Width of 5: Yes slight more negative voltage peak further
    // Width of 8: Yes more neg even TxDAC:0x0f on 3.5MHz Comp:
    // Width of 12: Yes, even more so, on target
    monostable #(
      `ifndef DEMO_VAR_FREQ_PULSE
        .PULSE_WIDTH(12),         // 80MHz / 7MHz (for 2x3.5MHz for half-cycle on hold)
      `endif
      `ifdef DEMO_VAR_FREQ_PULSE
        .PULSE_WIDTH(11),
      `endif
      .COUNTER_WIDTH(5)           // 5 wires for up to count of 32
    ) msv_delay_to_rtz (
      .clk        (SAMPLECLK),
      .reset      (trig_in_rise),
      .trigger    (!pulse_on),    // does this work?
      .pulse      (delay_to_rtz)
    );

    // Indeed on scope, return to zero starts exactly with the application of this pulse
    monostable #(
      `ifndef DEMO_VAR_FREQ_PULSE
        .PULSE_WIDTH(64),         // Long (about 4x = 2 cycles) for the RTZ
      `endif
      `ifdef DEMO_VAR_FREQ_PULSE
        .PULSE_WIDTH(10),
      `endif
        .COUNTER_WIDTH(7)         // Max of 128 - if using the 1usec delay before SSR blanking - we have plenty of time
    ) msv_pulse_rtz (
        .clk      (SAMPLECLK),
        .reset    (trig_in_rise),
        .trigger  (!delay_to_rtz),  // does this work? do we need a combined signal to avoid the early trig?
        .pulse    (pulse_rtz)
    );

    // End of Standard Single Pulse
    //

`endif // end of NOT using DEMO_VAR_FREQ_PULSE









`ifdef DEMO_VAR_FREQ_PULSE

    wire
      pulse_on1, pulse_rtz1,
      pulse_on2, pulse_rtz2, pulse_on3, pulse_rtz3, pulse_on4, pulse_rtz4,
      pulse_on5, pulse_rtz5, pulse_on6, pulse_rtz6, pulse_on7, pulse_rtz7,
      pulse_on8, pulse_rtz8, pulse_on9, pulse_rtz9, pulse_on10, pulse_rtz10,
      pulse_on11, pulse_rtz11, pulse_on12, pulse_rtz12,
      pulse_rtzlast;

    assign pulse_on = (
        pulse_on1 || pulse_on2 || pulse_on3 || pulse_on4
        || pulse_on5 || pulse_on6 || pulse_on7 || pulse_on8
        || pulse_on9 // || pulse_on10 || pulse_on11 || pulse_on12
      );
    assign pulse_rtz = (
        pulse_rtz1 || pulse_rtz2 || pulse_rtz3 || pulse_rtz4 || pulse_rtzlast
        || pulse_rtz5 || pulse_rtz6 || pulse_rtz7 || pulse_rtz8
        || pulse_rtz9 // || pulse_rtz10 || pulse_rtz11 || pulse_rtz12
      );

    // Notes:
    // For demo VAR_FREQ / swept frequency:
    // We use: 2x faster clock and only the pulse and rtz sections of the control

    // Yes, this can definitely be parameterized and for loop equivalent ...
    // This is a very quick dev rev just to test and demo hardware capabilities
    // (ie not a demo of good coding practice - under the gun here that is ...)

    //
    // First pulse for VAR_FREQ demo idea
    monostable #(
      .PULSE_WIDTH(100), // was 24
      .COUNTER_WIDTH(8) // was 5
    ) msv_pulse_on1 (
      .clk        (clk160),
      .reset      (trig_in_rise),
      .trigger    (trig_in_rise),
      .pulse      (pulse_on1)
    );
    monostable #(
        .PULSE_WIDTH(70),   // was 23
        .COUNTER_WIDTH(8)   // was 5
    ) msv_pulse_rtz1 (
        .clk      (clk160),
        .reset    (trig_in_rise),
        .trigger  (!pulse_on1),
        .pulse    (pulse_rtz1)
    );
    // End of First Pulse
    //


    //
    // Second Pulse
    monostable #(
      .PULSE_WIDTH(50),     // was 22
      .COUNTER_WIDTH(7)     // was 5
    ) msv_pulse_on2 (
      .clk        (clk160),
      .reset      (trig_in_rise),
      .trigger    (!pulse_rtz1),
      .pulse      (pulse_on2)
    );
    monostable #(
        .PULSE_WIDTH(40),
        .COUNTER_WIDTH(7)
    ) msv_pulse_rtz2 (
        .clk      (clk160),
        .reset    (trig_in_rise),
        .trigger  (!pulse_on2),
        .pulse    (pulse_rtz2)
    );
    // End of Second Pulse
    //



    //
    // Third Pulse
    monostable #(
      .PULSE_WIDTH(35),
      .COUNTER_WIDTH(7)
    ) msv_pulse_on3 (
      .clk        (clk160),
      .reset      (trig_in_rise),
      .trigger    (!pulse_rtz2),
      .pulse      (pulse_on3)
    );
    monostable #(
        .PULSE_WIDTH(30),
        .COUNTER_WIDTH(7)
    ) msv_pulse_rtz3 (
        .clk      (clk160),
        .reset    (trig_in_rise),
        .trigger  (!pulse_on3),
        .pulse    (pulse_rtz3)
    );
    // End of Third Pulse
    //


    //
    // Pulse 4
    monostable #(
      .PULSE_WIDTH(26),
      .COUNTER_WIDTH(7)
    ) msv_pulse_on4 (
      .clk        (clk160),
      .reset      (trig_in_rise),
      .trigger    (!pulse_rtz3),
      .pulse      (pulse_on4)
    );
    monostable #(
        .PULSE_WIDTH(22),
        .COUNTER_WIDTH(7)
    ) msv_pulse_rtz4 (
        .clk      (clk160),
        .reset    (trig_in_rise),
        .trigger  (!pulse_on4),
        .pulse    (pulse_rtz4)
    );
    // End of Pulse 4
    //


    //
    // Pulse 5
    monostable #(
      .PULSE_WIDTH(18),
      .COUNTER_WIDTH(7)
    ) msv_pulse_on5 (
      .clk        (clk160),
      .reset      (trig_in_rise),
      .trigger    (!pulse_rtz4),
      .pulse      (pulse_on5)
    );
    monostable #(
        .PULSE_WIDTH(16),
        .COUNTER_WIDTH(7)
    ) msv_pulse_rtz5 (
        .clk      (clk160),
        .reset    (trig_in_rise),
        .trigger  (!pulse_on5),
        .pulse    (pulse_rtz5)
    );
    // End of Pulse 5
    //



    //
    // Pulse 6
    monostable #(
      .PULSE_WIDTH(16),
      .COUNTER_WIDTH(7)
    ) msv_pulse_on6 (
      .clk        (clk160),
      .reset      (trig_in_rise),
      .trigger    (!pulse_rtz5),
      .pulse      (pulse_on6)
    );
    monostable #(
        .PULSE_WIDTH(14),
        .COUNTER_WIDTH(7)
    ) msv_pulse_rtz6 (
        .clk      (clk160),
        .reset    (trig_in_rise),
        .trigger  (!pulse_on6),
        .pulse    (pulse_rtz6)
    );
    // End of Pulse 6
    //




    //
    // Pulse 7
    monostable #(
      .PULSE_WIDTH(14),
      .COUNTER_WIDTH(5)
    ) msv_pulse_on7 (
      .clk        (clk160),
      .reset      (trig_in_rise),
      .trigger    (!pulse_rtz6),
      .pulse      (pulse_on7)
    );
    monostable #(
        .PULSE_WIDTH(12),
        .COUNTER_WIDTH(5)
    ) msv_pulse_rtz7 (
        .clk      (clk160),
        .reset    (trig_in_rise),
        .trigger  (!pulse_on7),
        .pulse    (pulse_rtz7)
    );
    // End of Pulse 7
    //




    //
    // Pulse 8
    monostable #(
      .PULSE_WIDTH(10),
      .COUNTER_WIDTH(4)
    ) msv_pulse_on8 (
      .clk        (clk160),
      .reset      (trig_in_rise),
      .trigger    (!pulse_rtz7),
      .pulse      (pulse_on8)
    );
    monostable #(
        .PULSE_WIDTH(10),
        .COUNTER_WIDTH(5)
    ) msv_pulse_rtz8 (
        .clk      (clk160),
        .reset    (trig_in_rise),
        .trigger  (!pulse_on8),
        .pulse    (pulse_rtz8)
    );
    // End of Pulse 8
    //



    //
    // Pulse 9
    monostable #(
      .PULSE_WIDTH(8),
      .COUNTER_WIDTH(5)
    ) msv_pulse_on9 (
      .clk        (clk160),
      .reset      (trig_in_rise),
      .trigger    (!pulse_rtz8),
      .pulse      (pulse_on9)
    );
    monostable #(
        .PULSE_WIDTH(8),
        .COUNTER_WIDTH(5)
    ) msv_pulse_rtz9 (
        .clk      (clk160),
        .reset    (trig_in_rise),
        .trigger  (!pulse_on9),
        .pulse    (pulse_rtz9)
    );
    // End of Pulse 9
    //


    //
    // Around here we need to update analog components on the board that
    // are in place to limit slope because some manufacturers make transducers
    // with inductor tuning that requires heavy damping to control the load
    // if we want to keep amplitude (just due to the intentionally limited)
    // slope
    //


    /*
    //
    // Pulse 10
    monostable #(
      .PULSE_WIDTH(16),
      .COUNTER_WIDTH(5)
    ) msv_pulse_on10 (
      .clk        (clk160),
      .reset      (trig_in_rise),
      .trigger    (!pulse_rtz9),
      .pulse      (pulse_on10)
    );
    monostable #(
        .PULSE_WIDTH(16),
        .COUNTER_WIDTH(5)
    ) msv_pulse_rtz10 (
        .clk      (clk160),
        .reset    (trig_in_rise),
        .trigger  (!pulse_on10),
        .pulse    (pulse_rtz10)
    );
    // End of Pulse 10
    //




    //
    // Pulse 11
    monostable #(
      .PULSE_WIDTH(16),
      .COUNTER_WIDTH(5)
    ) msv_pulse_on11 (
      .clk        (clk160),
      .reset      (trig_in_rise),
      .trigger    (!pulse_rtz10),
      .pulse      (pulse_on11)
    );
    monostable #(
        .PULSE_WIDTH(16),
        .COUNTER_WIDTH(5)
    ) msv_pulse_rtz11 (
        .clk      (clk160),
        .reset    (trig_in_rise),
        .trigger  (!pulse_on11),
        .pulse    (pulse_rtz11)
    );
    // End of Pulse 11
    //




    //
    // Pulse 12
    monostable #(
      .PULSE_WIDTH(16),
      .COUNTER_WIDTH(5)
    ) msv_pulse_on12 (
      .clk        (clk160),
      .reset      (trig_in_rise),
      .trigger    (!pulse_rtz11),
      .pulse      (pulse_on12)
    );
    monostable #(
        .PULSE_WIDTH(16),
        .COUNTER_WIDTH(5)
    ) msv_pulse_rtz12 (
        .clk      (clk160),
        .reset    (trig_in_rise),
        .trigger  (!pulse_on12),
        .pulse    (pulse_rtz12)
    );
    // End of Pulse 12
    //
    */


    //
    // Final RTZ
    monostable #(
        .PULSE_WIDTH(64),
        .COUNTER_WIDTH(7)
    ) msv_lastpulse_rtz (
        .clk      (SAMPLECLK),
        .reset    (trig_in_rise),
        .trigger  (!pulse_on9),     // Yes, we'll just overlap the above for now
        .pulse    (pulse_rtzlast)
    );
    //
    //



`endif // End of DEMO_VAR_FREQ_PULSE


    // End Pulse Ctrl
    //
    //








    //
    // Rx Delay control
    //
  `ifdef USE_RX_DELAY_CTRL_2BIT

    parameter RX_DELAY_BASE = 2400; // 80e6 (clk rate) X 30e-6 = 2400 where 30e-6 is increments of 30 microseconds

    wire rx_delay;
    reg [13:0] rx_delay_clocks;

    always @( posedge SAMPLECLK )
    begin
      if ( trig_in_rise ) begin
        rx_delay_clocks <= ( RX_DELAY_BASE * rx_delay_ctrl ) + 2; // See note below
      end
    end

    // 2 clock offset allows us to use the same !rx_delay signal to capture
    // the waveform even when the input is indicating zero usec delay request.
    // Using just +1 creates some jitter at the converter
    // quite visible - probably too fast a timing somewhere.
    // 2 clocks gets us into the 40MHz timing range, sufficiently low enough
    // for the converter's specs.

    monostable_vpw14b #(
        //.PULSE_WIDTH( RX_DELAY_BASE * rx_delay_ctrl_val ),
        .COUNTER_WIDTH(14)         // 2^14 = 16,384 max - for 3x rx delay base of 2400 => 7200
    ) msv_rx_delay (
        .pulse_width (rx_delay_clocks),
        .clk      (SAMPLECLK),
        .reset    (trig_in_rise),
        .trigger  (trig_in_rise),  // does this work? do we need a combined signal to avoid the early trig?
        .pulse    (rx_delay)
    );


  `endif // end USE RX DELAY 2 BIT SELECTOR







    //
    // MONOSTABLE - Main Capture
    //
    wire trigd;         // trigd = triggered =

    monostable #(
      .PULSE_WIDTH(NSAMPLES*2),
      .COUNTER_WIDTH(14)
    ) msv_capt(
      .clk          (SAMPLECLK),
      .reset        (trig_in_rise),
`ifndef USE_RX_DELAY_CTRL_2BIT
      .trigger      (trig_in_rise),
`endif
`ifdef USE_RX_DELAY_CTRL_2BIT
      .trigger      (!rx_delay),
`endif
      .pulse        (trigd)
    );


  `ifdef SELFREAD
    wire trigd_rise, trigd_fall;
    edge_detect edet_trigd (
        .async_sig  (trigd),
        .clk        (SAMPLECLK),
        .rise       (trigd_rise),
        .fall       (trigd_fall)
    );
  `endif


  `ifndef SELFREAD
  `ifdef USE_READ_MEM_IN_AS_READ_MEM_IN
    wire read_mem_rise, read_mem_fall;
    assign read_mem = read_mem_in;
    edge_detect edet_read_mem (
        .async_sig  (read_mem_in),
        .clk        (SAMPLECLK),
        .rise       (read_mem_rise),
        .fall       (read_mem_fall)
    );
  `endif
  `endif



  //`ifdef SELFREAD | SYNC_SELFREAD_WITH_CLOCKED_OUT
    // Create this signal regardless, not getting || macro ifdefs to work yet ...
    // At 1.25MHz output write strobe rate, half a cycle is 400ns clock pulse width
    // For an 80MHz clock (12.5ns full cycle), that is:
    // 400ns / 12.5 ns = 32 clocks
    // So yeah, to delay by 32 clocks or so would put us maybe right in the middle of
    // of the strobe
    // Use: 32 NOT SELFREAD ? (using nano)
    // Or 40MHz (data grab rate, 8-bits) / 1.25MHz (data comms out rate, 8-bits)
    // 40/1.25 = 32 ... 1/2 => 16
    // Use: 16 SELFREAD - good enough - still messy at proto wire stages ...
    // ok
    // Current HDL-0108-RSCPT design, max comms throughput in theory is 12MHz (12Mbps)
    // 1.25MHz x 8-bits => 10MHz => 10Mbps throughput
    wire read_mem_shifted;
    shift #(
        .DEPTH(16)
      ) read_mem_shifted_shift (
        .clk        (SAMPLECLK),
        .data_in    (read_mem),
        .data_out   (read_mem_shifted)
    );
  //`endif



  `ifndef SELFREAD
  `ifdef SYNC_SELFREAD_WITH_CLOCKED_OUT
    assign fifo_wri = !read_mem_shifted; // inverted logic
  `endif
  `endif




  `ifdef SELFREAD
    reg new_capt;
    assign read_mem = dumpclk; // & new_capt;
    assign fifo_wri = read_mem_shifted;
    //assign fifo_wri = read_mem;
    //assign read_mem_rise = !trigd & dumpclk & new_capt;
    wire read_mem_rise, read_mem_fall;
    edge_detect edet_self_read_mem (
        .async_sig  (read_mem),
        .clk        (SAMPLECLK),
        .rise       (read_mem_rise),
        .fall       (read_mem_fall)
    );
  `endif


    // https://electronics.stackexchange.com/questions/26502/verilog-check-for-two-negedges-in-always-block
    wire trig_in_fall;
    edge_detect edet_trig_in (
      .async_sig  (trig_in),
      .clk        (sclk), //(clk_copy), // so far SAMPLECLK is best
      .rise       (trig_in_rise),
      .fall       (trig_in_fall)
    );





    always @( posedge SAMPLECLK )
    begin
      if ( trig_in_rise ) begin
        addr_rd_nxt <= 12'b 0000_0000_0000;
      end
      if ( read_mem_rise ) begin
        addr_rd_nxt <= (addr_rd + 1'b 1);
      end
    end




  `ifdef SELFREAD
    always @( posedge SAMPLECLK )
    begin
      // TODO make this a falling edge of trigd sync signal? to keep it clean
      // because the trigd is the monostable pulse output
      if ( trigd_fall ) begin
        new_capt <= 1'b 1;
      end
      if ( addr_rd_nxt == NSAMPLES ) begin
        new_capt <= 1'b 0;
      end
    end
  `endif








    always @( posedge SAMPLECLK )
    begin
      if ( trig_in_rise ) begin
        addr_wr_nxt <= 12'b 0000_0000_0000;
      end else if ( trigd ) begin
        addr_wr_nxt <= (addr_wr + 1'b 1);
      end
    end







    // SOF - Start of Frame - Const section construction - always stays the same
    // seq_id Sequence ID ends up being base 1 - that is, first waveform back starts at
    // the number 1 (not zero) - and wraps at 255 - so really only a count range of 255 items before wrap to 1
    // TODO verify behavior after the RST signal, in terms of first WF seq_id
    // Until you update it ...
    // TODO - module?
    // And confirm
    reg [7:0] sof [0:11];               // 12-byte start of frame
    // For individual clock cycle timing verifications or auto sync, on the
    // serial comms output end, you may wish to use values like:
    // 0x55 or 0xaa for some sync or SOF bytes
    // However, 0x00 and 0xff so close in time quite definitely exceed the intended
    // allowable signal content and thus are easy to start with for SOF
    // delimiters.
    // aa = 170
    // 55 = 85
    initial begin
      sof[0] <= 8'h aa;
      sof[1] <= 8'h 55;
      sof[2] <= 8'h aa;
      sof[3] <= 8'h 55;

      sof[8] <= 8'h ff;
      sof[9] <= 8'h 00;
      sof[10] <= 8'h ff;
      sof[11] <= 8'h 00;
    end



    /*
    always @( posedge sclk )
    begin

      if ( trigd ) begin //do_capture ) begin
        addr_wr <= addr_wr_nxt;
        mem[addr_wr] <= data_in;      // To test correct sequencing: <= addr_wr[7:0] etc.
      end

    end
    */

    // This will give a half delay?
    wire delayed_capture_clk;
    // Delay until data good at the converter
    assign delayed_capture_clk = ( ( sclk && ~SAMPLECLK ) ); // && ~clk160 );

    always @( posedge delayed_capture_clk )
    begin

      if ( trigd ) begin
      //  addr_wr <= addr_wr_nxt;
        mem[addr_wr] <= data_in;
      end

    end


    always @( posedge SAMPLECLK )
    begin

      // TODO could rework this
      // Currently addressed via MCU n_reset signal timing
      // However, with prior n_reset timing from the MCU that is now an extended
      // active low signal that gets set back to high only just before
      // the next channel cycle starts over again
      // Otherwise, with prior on the MCU timing, sometimes the
      // seq_id_nxt reset to 00 is not captured persistently correctly
      // so channel numbers climb higher until the next sequence of 8 or 8*n

      if ( n_reset == 1'b 0 ) begin // if !RST (active low)
        seq_id_nxt <= 8'h 00;
      end else begin
        if ( trig_in_rise ) begin  // was not nested, rather end else if
          seq_id_nxt <= seq_id + 1;
        end
      end

      if ( trigd ) begin //do_capture ) begin
        addr_wr <= addr_wr_nxt;
        //mem[addr_wr] <= data_in;      // To test correct sequencing: <= addr_wr[7:0] etc.
      end

      if ( read_mem ) begin
        if ( addr_rd == 4 ) begin
          addr_rd <= addr_rd_nxt;
          seq_id <= seq_id_nxt;
          val <= seq_id;
        end else if ( addr_rd < 12 ) begin
          addr_rd <= addr_rd_nxt;
          val <= sof[addr_rd];
        end else begin
          addr_rd <= addr_rd_nxt;
          val <= mem[addr_rd];          // To test correct addr_rd handling: <= addr_rd[7:0] etc.
        end
      end

    end

    // Back to the LED flasher test code, left-overs
    // drive USB pull-up resistor to '0' to disable USB
    //assign USBPU = 0;
    // now not using this, to save a public pin
    //assign LED = trig_in;



    // Reset section
    // Assuming yes that we have control of the RST input, from external
    // circuitry - and yes, may be different from power up,
    // For example to reset to initial conditions or reset a sequence
    // ID after a block of captures.
    // Here, we will assume an active low RST signal, as without the function
    // explicitly set up on the HDL MCU or this UP5K device, it is pulled up to
    // the power rail (3.3 here)
    // Async reset, allows either signal to run the block: clock or RST
    // Sync reset - we use this because two conditions can drive into seq_id_nxt
    wire n_reset;
    assign n_reset = RST; // not using directly to allow a spot to interject an updated functionality
    //always @ ( posedge SAMPLECLK or negedge n_reset)  // Async
    // Thus: some hold time is required for the RST line external drive, but not much
    // Moving this section to the block above where these vars are otherwise used for the
    // remaining control flow ...
    /*
    always @ ( posedge SAMPLECLK )
    begin
      if ( n_reset == 1'b 0 ) begin // if !RST (active low)
        seq_id_nxt <= 8'h 00;
      end else if ( trig_in_rise ) begin
        seq_id_nxt <= seq_id + 1;
      end
    end
    */




    //
    //
    // UART Implementation - taken from:
    // See module section for sources and examples
    //
    // Testing:
    // Every 1 sec, send a next char over uart
    //

    `ifdef USE_LATTICE_BREAKOUT_DEMO
    `ifdef USE_UART

      parameter ASCII_0 = 8'd48; //8'd0; //8'd48;
      parameter ASCII_9 = 8'd57; //8'd255; //8'd57;

      // UART registers
      reg [7:0] uart_txbyte = ASCII_0;
      //reg uart_send = 1'b0;
      wire uart_send;
      wire uart_txed;


      //
      // Baud clock generation setup
      reg clk_baud_115200 = 0;
      reg [31:0] cntr_115200 = 32'b0;
      // 160MHz / 115200 = 1388.8888888
      // Divide by two since we are toggling
      // /2 = 694.44444 => 694 => % error =
      // 160M / 694 / 2 = 115,273.8 - 115200 / 115200 => 73.8 / 115200 => 0.06%
      parameter period_115200 = 694;

      always @ (posedge clk160) begin
          // baud clock
          cntr_115200 <= cntr_115200 + 1;
          if (cntr_115200 == period_115200) begin
              clk_baud_115200 <= ~clk_baud_115200;
              cntr_115200 <= 32'b0;
          end
      end


      // Standard baud rates:
      // 7.3728 MHz / 2^n
      // Max for currently used dev/test RS-485 to USB/Serial:
      // 7.3728 / 8 = 921.6kHz Baud rate (called 921k)
      reg clk_baud_921600 = 0;
      reg [31:0] cntr_921600 = 32'b0;
      parameter period_921600 = 87; // 160MHz/921600 = 173.6111 / 2 = 86.8
      // 174 works but 87 does not ... yet.

      always @ ( posedge clk160 ) begin

        cntr_921600 <= cntr_921600 + 1;
        if ( cntr_921600 == period_921600 ) begin
            clk_baud_921600 <= ~clk_baud_921600;
            cntr_921600 <= 32'b0;
        end

      end



      // Non-standard baud rates:
      // eg 10Mbps for FTDI if using baud rate aliasing
      // However, never got this to work (yet?) on Mac OS X
      // And, haven't yet tested with the agnostic D2xx type test code
      // However, using the standard baud rate multiples seems to work,
      // even including 2x 921600 = 1,843,200 entered as a custom value for port rate
      // in CoolTerm
      // And these results are for testing the PLL as on the UP5K Lattice Dev Kit
      // Thus, apparently the real clock rate is 159 MHz
      // Also, with the current UART implementation, the speed seems to max functionally
      // at about 3.57 MHz -- and then even higher than using a divisor of 16
      // or sometimes
      reg clk_baud_fastest = 0;
      reg [31:0] cntr_fastest = 32'b0;
      parameter period_fastest = 43;

      always @ ( posedge clk160 ) begin

        cntr_fastest <= cntr_fastest + 1;         // these were cntr_10M
        if ( cntr_fastest == period_fastest ) begin  // was cntr_10M == period_10M
            clk_baud_fastest <= ~clk_baud_fastest;  // clk_baud_10M
            cntr_fastest <= 32'b0;              // was cntr_10M
        end

      end


      wire clk_baud;
      //assign clk_baud = clk_baud_115200;
      //assign clk_baud = clk_baud_921600;
      //assign clk_baud = clk_baud_10M;
      assign clk_baud = clk_baud_fastest;


      //
      // 1-sec clock generation setup
      // Simpler earlier using: just the same clk160 block and a period of
      // e.g. 80,000,000 -- however kept running into issues --
      // 8,000,000 was functional -- generally glitchy though through ranges
      // stopped debugging and just reworked as a clk160 divided to clk5
      // and then using smaller numbers for period ()
      reg clk_1 = 1'b0;
      //reg [31:0] cntr_1 = 32'b0;
      // Divide 160MHz by 2 since we are toggling
      //parameter period_1 = 80000000; // For 1 Hz clock: 80,000,000 -- not yet clean // 8,000,000 clean so far is 10 Hz
      //reg [31:0] period_1 = 32'd20000000;

      wire clk5;
      divide_by_n #(.N(32)) div_clk5(clk160, 1'b0, clk5);
      //parameter period_1 = 5000000;
      reg [31:0] period_1 = 32'd5000000;
      reg [31:0] cntr_1 = 32'd5000000; //period_1; //32'b0;

      wire clk_uart_trigger_output;
      assign clk_uart_trigger_output = clk_1; //clk_baud_115200; //clk_1;


      // ifdef USE_UART_BAUD_921600
      //parameter period_tx_send_pulse = 4 * period_921600;

      // ifdef USE_UART_BAUD_10000000 (10 MHz)
      //parameter period_tx_send_pulse = 4 * period_10M;
      parameter period_tx_send_pulse = 4 * period_fastest;

      always @ (posedge clk5) begin
          cntr_1 <= cntr_1 - 1;
          if (cntr_1 == 32'b0) begin
              clk_1 <= ~clk_1;
              cntr_1 <= period_1;
          end
      end


      monostable #(
        .PULSE_WIDTH(period_tx_send_pulse),
        .COUNTER_WIDTH(32)
      ) msv_uart_send_pulse (
        .clk          (clk160),
        .reset        (RST),
        .trigger      (clk_uart_trigger_output),
        .pulse        (uart_send)
      );


      uart_tx_8n1 transmitter (

          // module input: baud rate clock (init test @ 115200)
          .clk (clk_baud),

          // module input: byte to be transmitted
          .txbyte (uart_txbyte),

          // module input: flag to enable sending data
          .senddata (uart_send),

          // output: tx is finished
          .txdone (uart_txed),

          // output: UART tx pin
          .tx (UART_TX)
      );


      /*
      reg trig_uart_tx_rise;
      reg trig_uart_tx_fall;
      edge_detect edet_trig_tx (
            .async_sig  (clk_uart_trigger_output),
            .clk        (clk_baud), // not working with 921600? or even at 115200 baud for this pulse, still no go at 921600 true baud
            .rise       (trig_uart_tx_rise),
            .fall       (trig_uart_tx_fall)
        );*/

      //assign uart_send = trig_uart_tx_rise;

      `ifndef FLASH_LEDS
        assign RGB0 = clk_1; //~uart_send;
        assign RGB1 = 1'b1;
        assign RGB2 = 1'b1;
      `endif

      always @ ( posedge clk_uart_trigger_output )
      begin

          if (uart_txbyte == ASCII_9) begin
              uart_txbyte <= ASCII_0;
          end else begin
              uart_txbyte <= uart_txbyte + 1;
          end

      end


    `endif
    `endif





endmodule
















// *****************************************************************************
//
// MODULES
//      MODULES
//          MODULES
//              MODULES
//                    MODULES
//
// *****************************************************************************















// *****************************************************************************
//
// UART
//
// UART Implementation - taken from various including:
//
// (A)
// https://www.fpga4fun.com/SerialInterface5.html
// https://www.fpga4fun.com/SiteInformation.html
// See async.v verilog example
// Both copyright: fpga4fun.com & KNJN LLC
//
// (B)
// See other example(s) at:
// (forum link here)
//
// (C)
// See Additional example:
// https://github.com/nesl/ice40_examples/blob/master/uart_transmission
//
// See Additional example:
// https://www.nandland.com/vhdl/modules/module-uart-serial-port-rs232.html
//
// *****************************************************************************


//////////////////////////////////////////////////////////
// fpga4fun & knjn.com
/*module serialGPIO2(
    input clk,
    input RxD,
    output TxD,

    output reg [7:0] GPout,  // general purpose outputs
    input [7:0] GPin  // general purpose inputs
);

  wire RxD_data_ready;
  wire [7:0] RxD_data;

  async_receiver RX(.clk(clk), .RxD(RxD), .RxD_data_ready(RxD_data_ready), .RxD_data(RxD_data));

  always @(posedge clk) if(RxD_data_ready) GPout <= RxD_data;

  async_transmitter TX(.clk(clk), .TxD(TxD), .TxD_start(RxD_data_ready), .TxD_data(GPin));

endmodule
//
//
// See copyright
// UART async Rx/Tx modules incl baud gen:
////////////////////////////////////////////////////////
// RS-232 RX and TX module
// (c) fpga4fun.com & KNJN LLC - 2003 to 2016

// The RS-232 settings are fixed
// TX: 8-bit data, 2 stop, no-parity
// RX: 8-bit data, 1 stop, no-parity (the receiver can accept more stop bits of course)

//`define SIMULATION   // in this mode, TX outputs one bit per clock cycle
                       // and RX receives one bit per clock cycle (for fast simulations)

////////////////////////////////////////////////////////
module async_transmitter(
	input clk,
	input TxD_start,
	input [7:0] TxD_data,
	output TxD,
	output TxD_busy
);

// Assert TxD_start for (at least) one clock cycle to start transmission of TxD_data
// TxD_data is latched so that it doesn't have to stay valid while it is being sent

parameter ClkFrequency = 25000000;	// 25MHz
parameter Baud = 115200;

generate
	if(ClkFrequency<Baud*8 && (ClkFrequency % Baud!=0)) ASSERTION_ERROR PARAMETER_OUT_OF_RANGE("Frequency incompatible with requested Baud rate");
endgenerate

////////////////////////////////
`ifdef SIMULATION
wire BitTick = 1'b1;  // output one bit per clock cycle
`else
wire BitTick;
BaudTickGen #(ClkFrequency, Baud) tickgen(.clk(clk), .enable(TxD_busy), .tick(BitTick));
`endif

reg [3:0] TxD_state = 0;
wire TxD_ready = (TxD_state==0);
assign TxD_busy = ~TxD_ready;

reg [7:0] TxD_shift = 0;
always @(posedge clk)
begin
	if(TxD_ready & TxD_start)
		TxD_shift <= TxD_data;
	else
	if(TxD_state[3] & BitTick)
		TxD_shift <= (TxD_shift >> 1);

	case(TxD_state)
		4'b0000: if(TxD_start) TxD_state <= 4'b0100;
		4'b0100: if(BitTick) TxD_state <= 4'b1000;  // start bit
		4'b1000: if(BitTick) TxD_state <= 4'b1001;  // bit 0
		4'b1001: if(BitTick) TxD_state <= 4'b1010;  // bit 1
		4'b1010: if(BitTick) TxD_state <= 4'b1011;  // bit 2
		4'b1011: if(BitTick) TxD_state <= 4'b1100;  // bit 3
		4'b1100: if(BitTick) TxD_state <= 4'b1101;  // bit 4
		4'b1101: if(BitTick) TxD_state <= 4'b1110;  // bit 5
		4'b1110: if(BitTick) TxD_state <= 4'b1111;  // bit 6
		4'b1111: if(BitTick) TxD_state <= 4'b0010;  // bit 7
		4'b0010: if(BitTick) TxD_state <= 4'b0011;  // stop1
		4'b0011: if(BitTick) TxD_state <= 4'b0000;  // stop2
		default: if(BitTick) TxD_state <= 4'b0000;
	endcase
end

assign TxD = (TxD_state<4) | (TxD_state[3] & TxD_shift[0]);  // put together the start, data and stop bits
endmodule


////////////////////////////////////////////////////////
module async_receiver(
  input clk,
	input RxD,
	output reg RxD_data_ready = 0,
	output reg [7:0] RxD_data = 0,  // data received, valid only (for one clock cycle) when RxD_data_ready is asserted

	// We also detect if a gap occurs in the received stream of characters
	// That can be useful if multiple characters are sent in burst
	//  so that multiple characters can be treated as a "packet"
	output RxD_idle,  // asserted when no data has been received for a while
	output reg RxD_endofpacket = 0  // asserted for one clock cycle when a packet has been detected (i.e. RxD_idle is going high)
);

parameter ClkFrequency = 25000000; // 25MHz
parameter Baud = 115200;

parameter Oversampling = 8;  // needs to be a power of 2
// we oversample the RxD line at a fixed rate to capture each RxD data bit at the "right" time
// 8 times oversampling by default, use 16 for higher quality reception

generate
	if(ClkFrequency<Baud*Oversampling) ASSERTION_ERROR PARAMETER_OUT_OF_RANGE("Frequency too low for current Baud rate and oversampling");
	if(Oversampling<8 || ((Oversampling & (Oversampling-1))!=0)) ASSERTION_ERROR PARAMETER_OUT_OF_RANGE("Invalid oversampling value");
endgenerate

////////////////////////////////
reg [3:0] RxD_state = 0;

`ifdef SIMULATION
wire RxD_bit = RxD;
wire sampleNow = 1'b1;  // receive one bit per clock cycle

`else
wire OversamplingTick;
BaudTickGen #(ClkFrequency, Baud, Oversampling) tickgen(.clk(clk), .enable(1'b1), .tick(OversamplingTick));

// synchronize RxD to our clk domain
reg [1:0] RxD_sync = 2'b11;
always @(posedge clk) if(OversamplingTick) RxD_sync <= {RxD_sync[0], RxD};

// and filter it
reg [1:0] Filter_cnt = 2'b11;
reg RxD_bit = 1'b1;

always @(posedge clk)
if(OversamplingTick)
begin
	if(RxD_sync[1]==1'b1 && Filter_cnt!=2'b11) Filter_cnt <= Filter_cnt + 1'd1;
	else
	if(RxD_sync[1]==1'b0 && Filter_cnt!=2'b00) Filter_cnt <= Filter_cnt - 1'd1;

	if(Filter_cnt==2'b11) RxD_bit <= 1'b1;
	else
	if(Filter_cnt==2'b00) RxD_bit <= 1'b0;
end

// and decide when is the good time to sample the RxD line
function integer log2(input integer v); begin log2=0; while(v>>log2) log2=log2+1; end endfunction
localparam l2o = log2(Oversampling);
reg [l2o-2:0] OversamplingCnt = 0;
always @(posedge clk) if(OversamplingTick) OversamplingCnt <= (RxD_state==0) ? 1'd0 : OversamplingCnt + 1'd1;
wire sampleNow = OversamplingTick && (OversamplingCnt==Oversampling/2-1);
`endif

// now we can accumulate the RxD bits in a shift-register
always @(posedge clk)
case(RxD_state)
	4'b0000: if(~RxD_bit) RxD_state <= `ifdef SIMULATION 4'b1000 `else 4'b0001 `endif;  // start bit found?
	4'b0001: if(sampleNow) RxD_state <= 4'b1000;  // sync start bit to sampleNow
	4'b1000: if(sampleNow) RxD_state <= 4'b1001;  // bit 0
	4'b1001: if(sampleNow) RxD_state <= 4'b1010;  // bit 1
	4'b1010: if(sampleNow) RxD_state <= 4'b1011;  // bit 2
	4'b1011: if(sampleNow) RxD_state <= 4'b1100;  // bit 3
	4'b1100: if(sampleNow) RxD_state <= 4'b1101;  // bit 4
	4'b1101: if(sampleNow) RxD_state <= 4'b1110;  // bit 5
	4'b1110: if(sampleNow) RxD_state <= 4'b1111;  // bit 6
	4'b1111: if(sampleNow) RxD_state <= 4'b0010;  // bit 7
	4'b0010: if(sampleNow) RxD_state <= 4'b0000;  // stop bit
	default: RxD_state <= 4'b0000;
endcase

always @(posedge clk)
if(sampleNow && RxD_state[3]) RxD_data <= {RxD_bit, RxD_data[7:1]};

//reg RxD_data_error = 0;
always @(posedge clk)
begin
	RxD_data_ready <= (sampleNow && RxD_state==4'b0010 && RxD_bit);  // make sure a stop bit is received
	//RxD_data_error <= (sampleNow && RxD_state==4'b0010 && ~RxD_bit);  // error if a stop bit is not received
end

`ifdef SIMULATION
assign RxD_idle = 0;
`else
reg [l2o+1:0] GapCnt = 0;
always @(posedge clk) if (RxD_state!=0) GapCnt<=0; else if(OversamplingTick & ~GapCnt[log2(Oversampling)+1]) GapCnt <= GapCnt + 1'h1;
assign RxD_idle = GapCnt[l2o+1];
always @(posedge clk) RxD_endofpacket <= OversamplingTick & ~GapCnt[l2o+1] & &GapCnt[l2o:0];
`endif

endmodule


////////////////////////////////////////////////////////
// dummy module used to be able to raise an assertion in Verilog
module ASSERTION_ERROR();
endmodule


////////////////////////////////////////////////////////
module BaudTickGen(
	input clk, enable,
	output tick  // generate a tick at the specified baud rate * oversampling
);
parameter ClkFrequency = 25000000;
parameter Baud = 115200;
parameter Oversampling = 1;

function integer log2(input integer v); begin log2=0; while(v>>log2) log2=log2+1; end endfunction
localparam AccWidth = log2(ClkFrequency/Baud)+8;  // +/- 2% max timing error over a byte
reg [AccWidth:0] Acc = 0;
localparam ShiftLimiter = log2(Baud*Oversampling >> (31-AccWidth));  // this makes sure Inc calculation doesn't overflow
localparam Inc = ((Baud*Oversampling << (AccWidth-ShiftLimiter))+(ClkFrequency>>(ShiftLimiter+1)))/(ClkFrequency>>ShiftLimiter);
always @(posedge clk) if(enable) Acc <= Acc[AccWidth-1:0] + Inc[AccWidth:0]; else Acc <= Inc[AccWidth:0];
assign tick = Acc[AccWidth];
endmodule


////////////////////////////////////////////////////////
// END of fpga4fun & knjn.com
//

*/



//
// https://github.com/nesl/ice40_examples/tree/master/uart_transmission
//
// file: uart_trx.v
//
// 8N1 UART Module, transmit only
//
module uart_tx_8n1 (
    clk,        // input clock
    txbyte,     // outgoing byte
    senddata,   // trigger tx
    txdone,     // outgoing byte sent
    tx,         // tx wire
    );

    /* Inputs */
    input clk;
    input[7:0] txbyte;
    input senddata;

    /* Outputs */
    output txdone;
    output tx;

    /* Parameters */
    parameter STATE_IDLE=8'd0;
    parameter STATE_STARTTX=8'd1;
    parameter STATE_TXING=8'd2;
    parameter STATE_TXDONE=8'd3;

    /* State variables */
    reg[7:0] state=8'b0;
    reg[7:0] buf_tx=8'b0;
    reg[7:0] bits_sent=8'b0;
    reg txbit=1'b1;
    reg txdone=1'b0;

    /* Wiring */
    assign tx=txbit;

    /* always */
    always @ (posedge clk) begin
        // start sending?
        if (senddata == 1 && state == STATE_IDLE) begin
            state <= STATE_STARTTX;
            buf_tx <= txbyte;
            txdone <= 1'b0;
        end else if (state == STATE_IDLE) begin
            // idle at high
            txbit <= 1'b1;
            txdone <= 1'b0;
        end

        // send start bit (low)
        if (state == STATE_STARTTX) begin
            txbit <= 1'b0;
            state <= STATE_TXING;
        end
        // clock data out
        if (state == STATE_TXING && bits_sent < 8'd8) begin
            txbit <= buf_tx[0];
            buf_tx <= buf_tx>>1;
            bits_sent <= bits_sent + 1;  // was =
        end else if (state == STATE_TXING) begin
            // send stop bit (high)
            txbit <= 1'b1;
            bits_sent <= 8'b0;
            state <= STATE_TXDONE;
        end

        // tx done
        if (state == STATE_TXDONE) begin
            txdone <= 1'b1;
            state <= STATE_IDLE;
        end

    end

endmodule
//
// top.v variant - renamed to sample_uart_git_module
/* Top level module for keypad + UART demo */
module sample_uart_git_top_module (
    // input hardware clock (12 MHz)
    hwclk,
    // all LEDs
    led1,
    // UART lines
    ftdi_tx,
    );

    /* Clock input */
    input hwclk;

    /* LED outputs */
    output led1;

    /* FTDI I/O */
    output ftdi_tx;

    /* 9600 Hz clock generation (from 12 MHz) */
    reg clk_9600 = 0;
    reg [31:0] cntr_9600 = 32'b0;
    parameter period_9600 = 625;

    /* 1 Hz clock generation (from 12 MHz) */
    reg clk_1 = 0;
    reg [31:0] cntr_1 = 32'b0;
    parameter period_1 = 6000000;

    // Note: could also use "0" or "9" below, but I wanted to
    // be clear about what the actual binary value is.
    parameter ASCII_0 = 8'd48;
    parameter ASCII_9 = 8'd57;

    /* UART registers */
    reg [7:0] uart_txbyte = ASCII_0;
    reg uart_send = 1'b1;
    wire uart_txed;

    /* LED register */
    reg ledval = 0;

    /* UART transmitter module designed for
       8 bits, no parity, 1 stop bit.
    */
    uart_tx_8n1 transmitter (
        // 9600 baud rate clock
        .clk (clk_9600),
        // byte to be transmitted
        .txbyte (uart_txbyte),
        // trigger a UART transmit on baud clock
        .senddata (uart_send),
        // input: tx is finished
        .txdone (uart_txed),
        // output UART tx pin
        .tx (ftdi_tx)
    );

    /* Wiring */
    assign led1=ledval;

    /* Low speed clock generation */
    always @ (posedge hwclk) begin
        /* generate 9600 Hz clock */
        cntr_9600 <= cntr_9600 + 1;
        if (cntr_9600 == period_9600) begin
            clk_9600 <= ~clk_9600;
            cntr_9600 <= 32'b0;
        end

        /* generate 1 Hz clock */
        cntr_1 <= cntr_1 + 1;
        if (cntr_1 == period_1) begin
            clk_1 <= ~clk_1;
            cntr_1 <= 32'b0;
        end
    end

    /* Increment ASCII digit and blink LED */
    always @ (posedge clk_1 ) begin
        ledval <= ~ledval;
        if (uart_txbyte == ASCII_9) begin
            uart_txbyte <= ASCII_0;
        end else begin
            uart_txbyte <= uart_txbyte + 1;
        end
    end

endmodule
//
// End of
//















// *****************************************************************************
//
// END OF UART MODULES SECTION
//
// *****************************************************************************











// *****************************************************************************
// PLL
// $ cd /Users/myself/.apio/packages/toolchain-icestorm/bin
// $ ./icepll -i 16 -o 80
// 16MHz in and 40MHz out:
// See also: https://github.com/YosysHQ/arachne-pnr/issues/64
// for CLK source pins and needed outputs
// https://github.com/mystorm-org/BlackIce-II/wiki/PLLs-Improved
// https://github.com/mystorm-org/BlackIce-II/wiki/PLLs-Advanced
//
// 160 MHz out below:
module pllcore(input clk, output clkin, clkout, lock);
	SB_PLL40_CORE #(
		.FEEDBACK_PATH("SIMPLE"),
		.PLLOUT_SELECT("GENCLK"),
		.DIVR(4'b0000),
		.DIVF(7'b0100111),
		.DIVQ(3'b010),        // 40MHz: 3'b100 -- vs -- 80MHz: 3'b011 -- vs -- 160MHz: 3'b010
		.FILTER_RANGE(3'b001)
	) uut (
		.LOCK(lock),
		.RESETB(1'b1),
		.BYPASS(1'b0),
		.REFERENCECLK(clk),
		.PLLOUTCORE(clkout)
	);
	assign clkin = clk;
endmodule
// 80 MHz out below:
/*module pllcore(input clk, output clkin, clkout, lock);
	SB_PLL40_CORE #(
		.FEEDBACK_PATH("SIMPLE"),
		.PLLOUT_SELECT("GENCLK"),
		.DIVR(4'b0000),
		.DIVF(7'b0100111),
		.DIVQ(3'b011),        // 40MHz: 3'b100 -- vs -- 80MHz: 3'b011
		.FILTER_RANGE(3'b001)
	) uut (
		.LOCK(lock),
		.RESETB(1'b1),
		.BYPASS(1'b0),
		.REFERENCECLK(clk),
		.PLLOUTCORE(clkout)
	);
	assign clkin = clk;
endmodule
*/
//
// cd ~/.apio/packages/toolchain-icestorm/bin
// ./icepll -i 12 -o 160
// For 12MHz in: USE_LATTICE_BREAKOUT_DEMO
// 160MHz out: 159MHz real
// 4'b0000
// 7'b0110100
// 3'b010
// F/R: 3'b001
//
// 16MHz in: HDL-0108-RSCPT series
// 4'b0000
// 7'b0100111
// 3'b010
// F/R: 3'b001
//
module pll2pad(input clk_in, output clkout_a, clkout_b, lock);
	SB_PLL40_2_PAD #(
		.FEEDBACK_PATH("SIMPLE"),
		.DIVR(4'b0000),
		.DIVF(7'b0110100),
		.DIVQ(3'b010),        // 40MHz: 3'b100 -- vs -- 80MHz: 3'b011 -- vs 160MHz: 3'b010 (For 16MHz in)
		.FILTER_RANGE(3'b001)
	) uut (
		.LOCK(lock),
		.RESETB(1'b1),
		.BYPASS(1'b0),
		.PACKAGEPIN(clk_in),
    .PLLOUTGLOBALB(clkout_b),
    //.PLLOUTCOREB(clkout_b),
		.PLLOUTCOREA(clkout_a)
	);
endmodule




module edge_detect (input async_sig,
                    input clk,
                    output reg rise,
                    output reg fall);

  reg [1:3] resync;

  always @(posedge clk)
  begin
    // detect rising and falling edges.
    rise <= resync[2] & !resync[3];
    fall <= resync[3] & !resync[2];
    // update history shifter.
    resync <= {async_sig , resync[1:2]};
  end

endmodule




module monostable (
        input clk,
        input reset,
        input trigger,
        output reg pulse = 0  // output reg pulse = 0
);

        parameter PULSE_WIDTH = 0;      // e.g. 4096 for 12-bit counter (0 to 4095 + 1)
        parameter COUNTER_WIDTH = 0;     // e.g. 13 to capture extra bit for 4096 sample intervals

        reg [COUNTER_WIDTH-1:0] count = 0; // TODO parameterize

        // In this case, we modify to ignore any reset signal
        // We just want trigger and then reset on reach max count
        wire count_rst = (count == PULSE_WIDTH); //reset | (count == PULSE_WIDTH);

        always @ (posedge trigger, posedge count_rst) begin
                if (count_rst) begin
                        pulse <= 1'b0;
                end else begin
                        pulse <= 1'b1;
                end
        end

        always @ (posedge clk, posedge count_rst) begin
                if(count_rst) begin
                        count <= 0;
                end else begin
                        if(pulse) begin
                                count <= count + 1'b1;
                        end
                end
        end

endmodule




module monostable_vpw14b (
        input [13:0] pulse_width,
        input clk,
        input reset,
        input trigger,
        output reg pulse = 0  // output reg pulse = 0
);

        //parameter PULSE_WIDTH = 0;      // e.g. 4096 for 12-bit counter (0 to 4095 + 1)
        parameter COUNTER_WIDTH = 0;     // e.g. 13 to capture extra bit for 4096 sample intervals

        reg [COUNTER_WIDTH-1:0] count = 0; // TODO parameterize

        // In this case, we modify to ignore any reset signal
        // We just want trigger and then reset on reach max count
        wire count_rst = (count == pulse_width); //reset | (count == PULSE_WIDTH);

        always @ (posedge trigger, posedge count_rst) begin
                if (count_rst) begin
                        pulse <= 1'b0;
                end else begin
                        pulse <= 1'b1;
                end
        end

        always @ (posedge clk, posedge count_rst) begin
                if(count_rst) begin
                        count <= 0;
                end else begin
                        if(pulse) begin
                                count <= count + 1'b1;
                        end
                end
        end

endmodule



module divide_by_n(
	input clk,
	input reset,
	output reg out
);

	parameter N = 2;

	reg [`CLOG2(N)-1:0] counter;

	always @(posedge clk)
	begin
		out <= 0;

		if (reset)
			counter <= 0;
		else
		if (counter == 0)
		begin
			out <= 1;
			counter <= N - 1;
		end else
			counter <= counter - 1;
	end

endmodule





module shift
(
  input clk,
  input data_in,
  output data_out
);

parameter DEPTH = 3;
reg [DEPTH-1:0] holding_register;

always @ (posedge clk) begin
  holding_register <= {holding_register[DEPTH-2:0], data_in};
end

assign data_out = holding_register[DEPTH-1];

endmodule
