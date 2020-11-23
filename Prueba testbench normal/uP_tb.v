/* Margareth Vela
Carné 19458
Sección: 20
*/

module testbench();

  reg clock, reset;
  reg [3:0]pushbuttons;
  wire phase, c_flag, z_flag;
  wire [3:0]instr, oprnd, data_bus, FF_out, accu;
  wire [7:0]program_byte;
  wire [11:0]PC, address_RAM;

  uP uP_proyecto(clock, reset, pushbuttons, phase, c_flag, z_flag, instr, oprnd, data_bus, FF_out, accu, program_byte, PC, address_RAM);

  always
    #5 clock <=~clock;

    initial begin
    $display(" \Procesador - Nibbler ");
    $display("clock reset pushbuttons |  phase  c_flag   z_flag   instr   oprnd   data_bus  FF_out   Accu   program_byte       PC          address_RAM ");
    $display("------------------------|----------------------------------------------------------------------------------------------------------------");
    $monitor("%b       %b       %b    |   %b       %b        %b      %b    %b     %b      %b    %b     %b     %b    %b ", clock, reset, pushbuttons, phase, c_flag, z_flag, instr, oprnd, data_bus, FF_out, accu, program_byte, PC, address_RAM);

      reset = 1; clock = 0; pushbuttons = 4'b0;
  #1  reset = 0; pushbuttons = 4'b0110;
  #242 pushbuttons = 4'b1110;
    end

    initial
      #1100 $finish;
    initial begin
        $dumpfile("uP_tb.vcd");
        $dumpvars(0, testbench);
      end
  endmodule
