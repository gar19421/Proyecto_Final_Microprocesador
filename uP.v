//Brandon Garrido
//Carnet: 19421
// Ing. Mecatrónica

//Flip-Flop tipo D de 1 bit
module FF_1BIT (input wire clk, reset, enable, input wire D, output reg Q);
	always @ (posedge clk or posedge reset or enable)begin
		if (reset) begin //si resete==1
			Q <= 1'b0; //Q es cero
		end
    else if (enable) begin // sino si enable==1 -> Q = D
      Q <= D;//unblocking assignament
    end
	end
endmodule


//Program counter con contador con enable de 12 bits
module Program_Counter (input wire clk, reset, enable, load, input wire [11:0]loadPC, output reg [11:0]PC);
  always @ (posedge clk, posedge reset) begin
    if (reset) begin
      PC <= 12'b0;//valor inicial del PC tras el reset
    end
    else if (load) begin//si se activa el enable del load
      PC <= loadPC;     //se carga el loadPC de la RAM
    end
    else if (enable) begin// sino si el enable del counter==1 el contador aumenta en 1
      PC <= PC + 1;
    end
  end
endmodule

//Program ROM con memory.list hex
module Program_ROM (input wire [11:0] PC, output wire [7:0] program_byte);
  reg [7:0] mem [0:4095];// creacion de arreglo de memoria
  initial begin
      $readmemh("memory.list", mem);//lectura del archivo de datos
  end
  assign program_byte = mem[PC];// resultado leido de la memoria
endmodule//program rom implementada junto con una lista de datos en el memory list


//Fetch con Flip-Flip tipo D de 8 bits
module Fetch (input wire clk, reset, enable, input wire [7:0] program_byte, output wire [3:0] instr, oprnd);
  FF_1BIT FF1(clk, reset, enable, program_byte[7], instr[3]);//4bits más significativos para el instr
  FF_1BIT FF2(clk, reset, enable, program_byte[6], instr[2]);
  FF_1BIT FF3(clk, reset, enable, program_byte[5], instr[1]);
  FF_1BIT FF4(clk, reset, enable, program_byte[4], instr[0]);
  FF_1BIT FF5(clk, reset, enable, program_byte[3], oprnd[3]);//4bits menos significativos para el oprn
  FF_1BIT FF6(clk, reset, enable, program_byte[2], oprnd[2]);
  FF_1BIT FF7(clk, reset, enable, program_byte[1], oprnd[1]);
  FF_1BIT FF8(clk, reset, enable, program_byte[0], oprnd[0]);
endmodule


//Banderas de Carry on y cero con FF tipo D de 2 bits
module Flags(input wire clk, reset, enable, input wire [1:0] D, output wire [1:0] C_Z);
    FF_1BIT FF1(clk, reset, enable, D[1], C_Z[1]);//bit más significativo el del carry
    FF_1BIT FF2(clk, reset, enable, D[0], C_Z[0]);
endmodule

//Flip Flop Tipo T de 1 bit para el phase
module Phase(input wire clk, reset, enable, output wire phase);
  FF_1BIT F1(clk, reset, enable, ~phase, phase);
endmodule

// Decode con sentencia casex dentro de un bloque always
module Decode(input wire [6:0] Address, output reg [12:0] Control_Signals);
  always @(Address) begin
    casex(Address)//casex con la implementación de los casos de las 21 instrucciones
      7'bxxxxxx0: Control_Signals = 13'b1000000001000 ;//caso 1
      7'b00001x1: Control_Signals = 13'b0100000001000 ;//caso 2
      7'b00000x1: Control_Signals = 13'b1000000001000 ;//caso 3
      7'b00011x1: Control_Signals = 13'b1000000001000 ;//caso 4
      7'b00010x1: Control_Signals = 13'b0100000001000 ;//caso 5
      7'b0010xx1: Control_Signals = 13'b0001001000010 ;//caso 6
      7'b0011xx1: Control_Signals = 13'b1001001100000 ;//caso 7
      7'b0100xx1: Control_Signals = 13'b0011010000010 ;//caso 8
      7'b0101xx1: Control_Signals = 13'b0011010000100 ;//Caso 9
      7'b0110xx1: Control_Signals = 13'b1011010100000 ;//caso 10
      7'b0111xx1: Control_Signals = 13'b1000000111000 ;//caso 11
      7'b1000x11: Control_Signals = 13'b0100000001000 ;//caso 12
      7'b1000x01: Control_Signals = 13'b1000000001000 ;//caso 13
      7'b1001x11: Control_Signals = 13'b1000000001000 ;//caso 14
      7'b1001x01: Control_Signals = 13'b0100000001000 ;//caso 15
      7'b1010xx1: Control_Signals = 13'b0011011000010 ;//caso 16
      7'b1011xx1: Control_Signals = 13'b1011011100000 ;//caso 17
      7'b1100xx1: Control_Signals = 13'b0100000001000 ;//caso 18
      7'b1101xx1: Control_Signals = 13'b0000000001001 ;//caso 19
      7'b1110xx1: Control_Signals = 13'b0011100000010 ;//caso 20
      7'b1111xx1: Control_Signals = 13'b1011100100000 ;//caso 21
      default:    Control_Signals = 13'b0 ; // caso por default
    endcase
  end
endmodule

//Bus driver implementado con buffer triestado de 4 bits
module Bus_Driver(input wire enable, input wire [3:0] oprnd, output wire [3:0] data_bus);
  assign data_bus = enable ? oprnd : 4'bz;//enable o alta impedancia
endmodule

//ALU implementada con sentencia case con las 5 operaciones
module ALU(input wire [3:0] A, B, input wire [2:0] S, output reg [3:0] Y, output reg [1:0] C_Z);
  reg [4:0] result;

  always @(*) begin// operaciones de la ALU
    case(S)
      3'b000:   result = A;//ST
      3'b001:   result = A - B;//COMPI
      3'b010:   result = B;//LIT
      3'b011:   result = A + B;//ADDI
      3'b100:   result = {1'b0, A ~& B};//NANDI
      default:  result = 5'b0;
    endcase

    assign Y = result[3:0];
    assign C_Z = { result[4] , ~(result[3] | result[2] | result[1] | result[0])};//asignacion de las banderas de cero y carry on

  end
endmodule

//Acumulador implementado con un FF tipo D de 4 bits
module Accumulator(input wire clk, reset, enable, input wire [3:0] D, output wire [3:0] Accu) ;
  FF_1BIT FF1(clk, reset, enable, D[3], Accu[3]);
  FF_1BIT FF2(clk, reset, enable, D[2], Accu[2]);
  FF_1BIT FF3(clk, reset, enable, D[1], Accu[1]);
  FF_1BIT FF4(clk, reset, enable, D[0], Accu[0]);
endmodule


//RAM Read, write and enable implementada con bloques always
module RAM (input wire enable, write, read, input wire [11:0]address_RAM, inout [3:0]data);//inout lectura-escritura
  reg [3:0] RAM [0:4095];
  reg [3:0] data_out;

  assign data = (enable & read & ~write) ? data_out:4'bz;// modo lectura o alta impedancia

  always @ ( address_RAM or data or enable or write ) begin//escritura en la RAM
    if (enable && write) begin
      RAM[address_RAM] = data;
    end
  end

  always @ ( address_RAM or enable or write or read) begin//lectura de la ram
    if (enable && ~write && read) begin
      data_out = RAM[address_RAM];
    end
  end
endmodule


//Salida del procesador implementada con un FF de 4 bits
module FF_out(input wire clk, reset, enable, input wire [3:0] D, output wire [3:0] FF_out) ;
  FF_1BIT FF1(clk, reset, enable, D[3], FF_out[3]);
  FF_1BIT FF2(clk, reset, enable, D[2], FF_out[2]);
  FF_1BIT FF3(clk, reset, enable, D[1], FF_out[1]);
  FF_1BIT FF4(clk, reset, enable, D[0], FF_out[0]);
endmodule// acumulador implementado con un fliflop de 4 bits

// union de todos los modulos del microprocesador
module uP(input wire clock, reset, input wire [3:0] pushbuttons,
  output wire phase, c_flag, z_flag, output wire [3:0] instr, oprnd, accu, data_bus, FF_out,
  output wire [7:0] program_byte, output wire [11:0] PC, address_RAM);//inputs y outputs del microprocesador

  wire [1:0] ALU_flags, C_Z;//declaración de wires
  wire [12:0] Control_Signals;
  wire [6:0] Adress;
  wire [3:0] ALU_out;
  wire [6:0] Address;

  assign c_flag = C_Z[1];//asignación de las banderas a la salida
  assign z_flag = C_Z[0];//bit mas significativo para el carry y el menos para el zero
  assign address_RAM = {oprnd, program_byte};//concatenación del address_RAM
  assign Address = {instr, c_flag, z_flag, phase};//concatenación del Adress

	//Instanciación de módulos y conexión respectiva de los distintos cables de entrada y salida de caca elemento
  Program_Counter M_Program_Counter(clock, reset, Control_Signals[12], Control_Signals[11], address_RAM, PC);
  Program_ROM    M_Program_ROM(PC, program_byte);
  Fetch          M_Fetch(clock, reset, ~phase, program_byte, instr, oprnd);
  Phase          M_Phase(clock, reset, 1'b1, phase);
  Flags          M_Flags(clock, reset, Control_Signals[9], ALU_flags, C_Z);
  Decode         M_Decode(Address, Control_Signals);
  Accumulator    M_Accumulator(clock, reset, Control_Signals[10], ALU_out , accu);
  Bus_Driver     M_Bus_Driver1(Control_Signals[1], oprnd, data_bus);
  Bus_Driver     M_Bus_Driver2(Control_Signals[3], ALU_out, data_bus);
  RAM            M_RAM(Control_Signals[5], Control_Signals[4], Control_Signals[5], address_RAM, data_bus);
  ALU            M_ALU(accu, data_bus, Control_Signals[8:6], ALU_out, ALU_flags);
  FF_out         M_FF_out(clock, reset, Control_Signals[0], data_bus, FF_out);
  Bus_Driver     M_In(Control_Signals[2], pushbuttons, data_bus);

endmodule
