`timescale 1ns / 1ps
module ArithmeticLogicUnitSystem (
    // Clock
    input wire Clock,

    // RF 
    input wire [3:0] RF_RegSel,
    input wire [3:0] RF_ScrSel,
    input wire [2:0] RF_FunSel,
    input wire [2:0] RF_OutASel,
    input wire [2:0] RF_OutBSel,

    // ALU 
    input wire ALU_WF,
    input wire [4:0] ALU_FunSel,

    // ARF 
    input wire [2:0] ARF_RegSel,
    input wire [1:0] ARF_FunSel,
    input wire [1:0] ARF_OutCSel,
    input wire [1:0] ARF_OutDSel,

    // DR 
    input wire DR_E,
    input wire [1:0] DR_FunSel,

    // Memory 
    input wire Mem_WR,
    input wire Mem_CS,

    // IR 
    input wire IR_LH,
    input wire IR_Write,

    // MUX 
    input wire [1:0] MuxASel,
    input wire [1:0] MuxBSel,
    input wire [1:0] MuxCSel,
    input wire MuxDSel,

    //outputs
    output wire [31:0] OutA,  
    output wire [31:0] OutB,  
    output wire [31:0] ALUOut,     
    output wire [3:0]  FlagsOut,   
    output wire [15:0] OutC,   
    output wire [15:0] OutD,   
    output wire [7:0]  MemOut,  
    output wire [15:0] IROut,      
    output wire [31:0] DROut,      
    output wire [15:0] Address     

);


    // Module outputs
    wire [31:0] RF_OutA;     
    wire [31:0] RF_OutB;      
    wire [31:0] ALU_Out;      
    wire [3:0]  ALU_FlagsOut; 
    wire [15:0] ARF_OutC;
    wire [15:0] ARF_OutD;     
    wire [31:0] DR_Out;      
    wire [7:0]  Mem_Out;      
    wire [15:0] IR_Out;      

    // MUX outputs 
    wire [31:0] MuxAOut;      
    wire [31:0] MuxBOut;      
    wire [7:0]  MuxCOut;      
    wire [31:0] MuxDOut;      


 

    // MUX d
    MUX_2TO1_32BIT MUXD (
        .sel(MuxDSel),
        .Y1(RF_OutA),        
        .Y2({16'b0, ARF_OutC}), 
        .Z(MuxDOut)          
    );

    // Register File
    RegisterFile RF (
        .Clock(Clock),
        .I(MuxAOut),          
        .RegSel(RF_RegSel),
        .ScrSel(RF_ScrSel),
        .FunSel(RF_FunSel),
        .OutASel(RF_OutASel),
        .OutBSel(RF_OutBSel),
        .OutA(RF_OutA),       
        .OutB(RF_OutB)      
    );

    // Arithmetic Logic Unit
    ArithmeticLogicUnit ALU (      
        .Clock(Clock),
        .A(MuxDOut),          
        .B(RF_OutB),       
        .FunSel(ALU_FunSel),
        .FlagsOut(ALU_FlagsOut),
        .ALUOut(ALU_Out),     
        .WF(ALU_WF)                
    );

    // MUX A
    MUX_4TO1_32BIT MUXA (
        .sel(MuxASel),
        .Y1(ALU_Out),
        .Y2({16'b0, ARF_OutC}), 
        .Y3(DR_Out),      
        .Y4({16'b0, IR_Out}),   
        .Z(MuxAOut)  
    );

    // MUX B
    MUX_4TO1_32BIT MUXB (
        .sel(MuxBSel),
        .Y1(ALU_Out),
        .Y2({16'b0, ARF_OutC}), 
        .Y3(DR_Out),           
        .Y4({16'b0, IR_Out}),  
        .Z(MuxBOut)   
    );

    // Address Register File 
    AddressRegisterFile ARF (
        .Clock(Clock),
        .I(MuxBOut), 
        .RegSel(ARF_RegSel),
        .FunSel(ARF_FunSel),
        .OutCSel(ARF_OutCSel),
        .OutDSel(ARF_OutDSel),
        .OutC(ARF_OutC),      
        .OutD(ARF_OutD)         
    );

    // Memory
    Memory MEM (
        .Address(ARF_OutD),
        .Data(ALU_Out[7:0]),
        .WR(Mem_WR),              
        .CS(Mem_CS),             
        .Clock(Clock),
        .MemOut(Mem_Out)      
    );

    // Instruction Register
    InstructionRegister IR (
        .I(Mem_Out),           
        .LH(IR_LH),                 
        .Write(IR_Write),           
        .Clock(Clock),
        .IROut(IR_Out)         
    );

    // Data Register 
    DataRegister DR (
        .I(Mem_Out),          
        .FunSel(DR_FunSel),
        .E(DR_E),   
        .Clock(Clock),
        .DROut(DR_Out)         
    );

    // MUX C
    MUX_4TO1_8BIT MUXC (
        .sel(MuxCSel),
        .Y1(ALU_Out[7:0]),
        .Y2(ALU_Out[15:8]),
        .Y3(ALU_Out[23:16]),
        .Y4(ALU_Out[31:24]),
        .Z(MuxCOut)          
    );

  
    assign OutA     = RF_OutA;
    assign OutB     = RF_OutB;
    assign ALUOut   = ALU_Out;
    assign FlagsOut = ALU_FlagsOut;
    assign OutC     = ARF_OutC;
    assign OutD     = ARF_OutD;
    assign MemOut   = Mem_Out;
    assign IROut    = IR_Out;
    assign DROut    = DR_Out;
    assign Address  = ARF_OutD; 

endmodule



module MUX_4TO1_32BIT (
    input wire [1:0] sel,
    input wire [31:0] Y1,
    input wire [31:0] Y2,
    input wire [31:0] Y3,
    input wire [31:0] Y4,
    output reg [31:0] Z 
);

    always @(*) begin
        case (sel)
            2'b00: Z = Y1;
            2'b01: Z = Y2;
            2'b10: Z = Y3;
            2'b11: Z = Y4;
            default: Z = 32'bx;
        endcase
    end
endmodule



module MUX_4TO1_8BIT (
    input wire [1:0] sel,
    input wire [7:0] Y1,
    input wire [7:0] Y2,
    input wire [7:0] Y3,
    input wire [7:0] Y4,
    output reg [7:0] Z 
);

    always @(*) begin 
        case (sel)
            2'b00: Z = Y1;
            2'b01: Z = Y2;
            2'b10: Z = Y3;
            2'b11: Z = Y4;
            default: Z = 8'bx; 
        endcase
    end
endmodule


`timescale 1ns / 1ps
module MUX_2TO1_32BIT (
    input wire sel,
    input wire [31:0] Y1,
    input wire [31:0] Y2,
    output reg [31:0] Z 
);

    always @(*) begin
        case (sel)
            1'b0: Z = Y1;
            1'b1: Z = Y2;
            default: Z = 32'bx; 
        endcase
    end
endmodule