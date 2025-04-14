// Corrected ArithmeticLogicUnitSystem
`timescale 1ns / 1ps
module ArithmeticLogicUnitSystem (
    // Clock
    input wire Clock,

    // RF Inputs
    input wire [3:0] RF_RegSel,
    input wire [3:0] RF_ScrSel,
    input wire [2:0] RF_FunSel,
    input wire [2:0] RF_OutASel,
    input wire [2:0] RF_OutBSel,

    // ALU Inputs
    input wire ALU_WF,
    input wire [4:0] ALU_FunSel,

    // ARF Inputs
    input wire [2:0] ARF_RegSel,
    input wire [1:0] ARF_FunSel,
    input wire [1:0] ARF_OutCSel,
    input wire [1:0] ARF_OutDSel,

    // DR Inputs
    input wire DR_E,
    input wire [1:0] DR_FunSel,

    // Memory Inputs
    input wire Mem_WR,
    input wire Mem_CS,

    // IR Inputs
    input wire IR_LH,
    input wire IR_Write,

    // MUX Inputs
    input wire [1:0] MuxASel,
    input wire [1:0] MuxBSel,
    input wire [1:0] MuxCSel,
    input wire MuxDSel,

    // --- Outputs needed by Testbench ---
    output wire [31:0] OutA,       // RF Output A
    output wire [31:0] OutB,       // RF Output B
    output wire [31:0] ALUOut,     // ALU Result Output
    output wire [3:0]  FlagsOut,   // ALU Flags Output (Z,C,N,O)
    output wire [15:0] OutC,       // ARF Output C
    output wire [15:0] OutD,       // ARF Output D
    output wire [7:0]  MemOut,     // Memory Output
    output wire [15:0] IROut,      // Instruction Register Output
    output wire [31:0] DROut,      // Data Register Output
    output wire [15:0] Address     // Memory Address Output (from ARF OutD)
    // Note: Mux outputs (MuxAOut etc.) are checked via hierarchical names in TB
);

    // --- Internal Wires ---
    wire [31:0] RF_OutA;      // RF OutA internal signal
    wire [31:0] RF_OutB;      // RF OutB internal signal
    wire [31:0] ALU_Out;      // ALU Result internal signal
    wire [3:0]  ALU_Fla; // ALU Flags internal signal
    wire [15:0] ARF_OutC;     // ARF OutC internal signal
    wire [15:0] ARF_OutD;     // ARF OutD internal signal (used for Address)
    wire [31:0] DR_Out;       // DR Output internal signal
    wire [7:0]  Mem_Out;      // Memory Output internal signal
    wire [15:0] IR_Out;       // IR Output internal signal

    // MUX Outputs (Inputs to Registers/ALU)
    wire [31:0] MuxAOut;      // MUX A Output -> RF Input I
    wire [31:0] MuxBOut;      // MUX B Output -> ARF Input I
    wire [7:0]  MuxCOut;      // MUX C Output -> Memory Address? No, Address from ARF_OutD. MUX C not directly used? Check Figure 11 again. MUX C output is not shown connected anywhere. Let's keep it but note it's unused based on diagram paths shown.
    wire [31:0] MuxDOut;      // MUX D Output -> ALU Input A


    // --- Module Instantiations ---

    // MUX D: Selects ALU Input A
    MUX_2TO1_32BIT MUXD (
        .sel(MuxDSel),
        .Y1(RF_OutA),          // Input 0: RF OutA
        .Y2({16'b0, ARF_OutC}), // Input 1: ARF OutC (Zero Extended)
        .Z(MuxDOut)            // Output -> ALU Input A
    );

    // Register File (RF)
    RegisterFile RF (
        .Clock(Clock),
        .I(MuxAOut),           // Input from MUX A
        .RegSel(RF_RegSel),
        .ScrSel(RF_ScrSel),
        .FunSel(RF_FunSel),
        .OutASel(RF_OutASel),
        .OutBSel(RF_OutBSel),
        .OutA(RF_OutA),        // Output A -> MUX D
        .OutB(RF_OutB)         // Output B -> ALU Input B
    );

    // Arithmetic Logic Unit (ALU)
    ArithmeticLogicUnit ALU (      // Instance name was ALU before, changed to _ALU
        .Clock(Clock),
        .A(MuxDOut),           // Input A from MUX D
        .B(RF_OutB),           // Input B from RF OutB
        .FunSel(ALU_FunSel),
        .FlagsOut(ALU_FlagsOut),// ALU Flags Output
        .ALUOut(ALU_Out),      // ALU Result Output
        .WF(ALU_WF)                 // Flags Write Enable
    );

    // MUX A: Selects input for Register File (RF)
    MUX_4TO1_32BIT MUXA (
        .sel(MuxASel),
        .Y1(ALU_Out),          // Input 00: ALU Result
        .Y2({16'b0, ARF_OutC}), // Input 01: ARF OutC (Zero Extended)
        .Y3(DR_Out),      // Input 10: Data Register Output
        .Y4({16'b0, IR_Out}),   // Input 11: Instruction Register Output (Zero Extended)
        .Z(MuxAOut)            // Output -> RF Input I
    );

    // MUX B: Selects input for Address Register File (ARF)
    MUX_4TO1_32BIT MUXB (
        .sel(MuxBSel),
        .Y1(ALU_Out),          // Input 00: ALU Result
        .Y2({16'b0, ARF_OutC}), // Input 01: ARF OutC (Zero Extended)
        .Y3(DR_Out),           // Input 10: Data Register Output
        .Y4({16'b0, IR_Out}),   // Input 11: Instruction Register Output (Zero Extended)
        .Z(MuxBOut)            // Output -> ARF Input I
    );

    // Address Register File (ARF)
    AddressRegisterFile ARF (
        .Clock(Clock),
        .I(MuxBOut),           // Input from MUX B
        .RegSel(ARF_RegSel),
        .FunSel(ARF_FunSel),
        .OutCSel(ARF_OutCSel),
        .OutDSel(ARF_OutDSel),
        .OutC(ARF_OutC),        // Output C -> MUX A, MUX B, MUX D
        .OutD(ARF_OutD)         // Output D -> Memory Address
    );

    // Memory
    Memory MEM (
        .Address(ARF_OutD),    // Address from ARF OutD
        .Data(ALU_Out[7:0]),   // Data Input (8-bit) from ALU Out LSB
        .WR(Mem_WR),                // Write Enable (0=Read, 1=Write)
        .CS(Mem_CS),                // Chip Select (0=Active)
        .Clock(Clock),
        .MemOut(Mem_Out)       // Memory Data Output -> DR, IR
    );

    // Instruction Register (IR)
    InstructionRegister IR (
        .I(Mem_Out),           // Input from Memory Out
        .LH(IR_LH),                 // Load High/Low byte select
        .Write(IR_Write),           // Write Enable
        .Clock(Clock),
        .IROut(IR_Out)         // Output -> MUX A, MUX B
    );

    // Data Register (DR)
    DataRegister DR (
        .I(Mem_Out),           // Input from Memory Out
        .FunSel(DR_FunSel),
        .E(DR_E),                   // Enable
        .Clock(Clock),
        .DROut(DR_Out)         // Output -> MUX A, MUX B
    );

    // MUX C: Selects 8-bit slice from ALU Output
    MUX_4TO1_8BIT MUXC (
        .sel(MuxCSel),
        .Y1(ALU_Out[7:0]),
        .Y2(ALU_Out[15:8]),
        .Y3(ALU_Out[23:16]),
        .Y4(ALU_Out[31:24]),
        .Z(MuxCOut)            // Output -> Unused based on diagram? Testbench checks ALUSys.MuxCOut hierarchically.
    );

    // --- Assign internal wires to outputs for Testbench ---
    assign OutA     = RF_OutA;
    assign OutB     = RF_OutB;
    assign ALUOut   = ALU_Out;
    assign FlagsOut = ALU_FlagsOut;
    assign OutC     = ARF_OutC;
    assign OutD     = ARF_OutD;
    assign MemOut   = Mem_Out;
    assign IROut    = IR_Out;
    assign DROut    = DR_Out;
    assign Address  = ARF_OutD; // Memory Address comes from ARF OutD

endmodule



module MUX_4TO1_32BIT (
    input wire [1:0] sel,
    input wire [31:0] Y1,
    input wire [31:0] Y2,
    input wire [31:0] Y3,
    input wire [31:0] Y4,
    output reg [31:0] Z // Changed to reg for assignment in always block
);

    always @(*) begin
        case (sel)
            2'b00: Z = Y1;
            2'b01: Z = Y2;
            2'b10: Z = Y3;
            2'b11: Z = Y4;
            default: Z = 32'bx; // Default assignment
        endcase
    end
endmodule

// Corrected MUX C

module MUX_4TO1_8BIT (
    input wire [1:0] sel,
    input wire [7:0] Y1,
    input wire [7:0] Y2,
    input wire [7:0] Y3,
    input wire [7:0] Y4,
    output reg [7:0] Z // Changed to reg for assignment in always block
);

    always @(*) begin // Fixed parenthesis
        case (sel)
            2'b00: Z = Y1;
            2'b01: Z = Y2;
            2'b10: Z = Y3;
            2'b11: Z = Y4;
            default: Z = 8'bx; // Default assignment
        endcase
    end
endmodule


`timescale 1ns / 1ps
module MUX_2TO1_32BIT (
    input wire sel,
    input wire [31:0] Y1,
    input wire [31:0] Y2,
    output reg [31:0] Z // Changed to reg for assignment in always block
);

    always @(*) begin
        case (sel)
            1'b0: Z = Y1;
            1'b1: Z = Y2; // Fixed missing semicolon
            default: Z = 32'bx; // Default assignment
        endcase
    end
endmodule