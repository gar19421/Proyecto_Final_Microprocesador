module testbench();
  //Variables de entrada
  reg clock, reset;
  reg [3:0]pushbuttons;

  //Variables de salida
  wire phase, c_flag, z_flag;
  wire [3:0]instr, oprnd, data_bus, FF_out, accu;
  wire [7:0]program_byte;
  wire [11:0]pc, address_ram;
  //Instanciaciones
  uP MicroProcessor(clock, reset, pushbuttons, phase, c_flag, z_flag, instr, oprnd, data_bus, FF_out, accu, program_byte, pc, address_ram);

  always//clock
    #5 clock <=~clock;

    initial begin
    #2
    $display(" \n MicroProcessor - Nibbler ");
    $display("clock reset pushbuttons |  phase  c_flag   z_flag   instr   oprnd   data_bus  FF_out   accu   program_byte       PC          address_RAM ");
    $display("------------------------|----------------------------------------------------------------------------------------------------------------");
    $monitor("%b       %b       %b    |   %b       %b        %b      %b    %b     %b      %b    %b     %b     %b    %b ", clock, reset, pushbuttons, phase, c_flag, z_flag, instr, oprnd, data_bus, FF_out, accu, program_byte, pc, address_ram);

      reset = 1; clock = 0; pushbuttons = 4'b0;
      #1  reset = 0; pushbuttons = 4'b0110;
      #242 pushbuttons = 4'b1110;
    end

    initial
      #1010 $finish;
    initial begin
        $dumpfile("uP_tb.vcd");
        $dumpvars(0, testbench);
      end
  endmodule
