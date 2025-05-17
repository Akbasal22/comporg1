`timescale 1ns / 1ps
module CPUSystem(
    input Clock,
    input Reset,
    output reg [11:0] T  // Added T as an output port
);
    // State counter
    reg [11:0] T_reg;  // Renamed internal T register to T_reg
    always @(*) begin 
        T = T_reg;
    end

    
    // Control signals for ALUSystem
    reg [2:0] RF_OutASel, RF_OutBSel;
    reg [2:0] RF_FunSel;
    reg [3:0] RF_RegSel, RF_ScrSel;
    reg [4:0] ALU_FunSel;
    reg ALU_WF; 
    reg [1:0] ARF_OutCSel, ARF_OutDSel, ARF_FunSel;
    reg [2:0] ARF_RegSel;
    reg IR_LH, IR_Write;
    reg Mem_WR, Mem_CS;
    reg [1:0] MuxASel, MuxBSel, MuxCSel;
    reg MuxDSel;
    reg DR_E;
    reg [1:0] DR_FunSel;

    reg T_Reset; //Used to reset T
    
    // Decoded register selections - needed for the errors you're getting
    reg [3:0] RF_RSel_Decoded;
    reg [3:0] DSTREG_Decoded;
    reg [2:0] SrcReg1_Decoded, SrcReg2_Decoded;
    
    // Flags from ALU
    wire Z, C, N, O;
    wire [3:0] FLAGS = {Z, C, N, O};
    
    // Flag bits for easier reference
    wire Z_Flag = Z;  // Zero flag
    wire C_Flag = C;  // Carry flag
    wire N_Flag = N;  // Negative flag
    wire O_Flag = O;  // Overflow flag
    
    // Instantiate ALUSystem from Project 1
    ArithmeticLogicUnitSystem ALUSys(
        .RF_OutASel(RF_OutASel),
        .RF_OutBSel(RF_OutBSel), 
        .RF_FunSel(RF_FunSel),
        .RF_RegSel(RF_RegSel), 
        .RF_ScrSel(RF_ScrSel),
        .ALU_FunSel(ALU_FunSel),
        .ALU_WF(ALU_WF), 
        .ARF_OutCSel(ARF_OutCSel), 
        .ARF_OutDSel(ARF_OutDSel), 
        .ARF_FunSel(ARF_FunSel),
        .ARF_RegSel(ARF_RegSel),
        .IR_LH(IR_LH), 
        .IR_Write(IR_Write), 
        .Mem_WR(Mem_WR), 
        .Mem_CS(Mem_CS),
        .MuxASel(MuxASel), 
        .MuxBSel(MuxBSel), 
        .MuxCSel(MuxCSel),
        .MuxDSel(MuxDSel),
        .DR_E(DR_E),
        .DR_FunSel(DR_FunSel),
        // .Z(Z), 
        // .C(C), 
        // .N(N), 
        // .O(O),
        .Clock(Clock)
    );
    
    // Extracted Opcode and instruction fields
    wire [5:0] Opcode = ALUSys.IROut[15:10];
    wire [1:0] RegSel = ALUSys.IROut[9:8];
    wire [7:0] Address = ALUSys.IROut[7:0];
    wire [2:0] DestReg = ALUSys.IROut[9:7];
    wire [2:0] SrcReg1 = ALUSys.IROut[6:4];
    wire [2:0] SrcReg2 = ALUSys.IROut[3:1];
    
    // Helper functions for register selection decoding
    
    // Function to decode RF register select based on RegSel (2-bit)
    function [3:0] GetRFRegSel;
        input [1:0] regNum;
        begin
            case(regNum)
                2'b00: GetRFRegSel = 4'b1000;  // R1
                2'b01: GetRFRegSel = 4'b0100;  // R2
                2'b10: GetRFRegSel = 4'b0010;  // R3
                2'b11: GetRFRegSel = 4'b0001;  // R4
            endcase
        end
    endfunction
    
    // Function to decode register selection for DestReg (3-bit)
    function [3:0] DecodeDSTREG;
        input [2:0] reg_code;
        begin
            case(reg_code)
                3'b100: DecodeDSTREG = 4'b1000;  // R1
                3'b101: DecodeDSTREG = 4'b0100;  // R2
                3'b110: DecodeDSTREG = 4'b0010;  // R3
                3'b111: DecodeDSTREG = 4'b0001;  // R4
                default: DecodeDSTREG = 4'b0000; // Not a register file register
            endcase
        end
    endfunction
    
    // Function to select ARF register based on 3-bit register code
    function [2:0] DecodeARFReg;
        input [2:0] reg_code;
        begin
            case(reg_code)
                3'b000: DecodeARFReg = 3'b100;  // PC
                3'b001: DecodeARFReg = 3'b010;  // SP
                3'b010: DecodeARFReg = 3'b001;  // AR
                3'b011: DecodeARFReg = 3'b001;  // AR (duplicate, as per spec)
                default: DecodeARFReg = 3'b000; // Not an ARF register
            endcase
        end
    endfunction
    
    // Function to get source register selection for ALU
    function [2:0] DecodeRFOutSel;
        input [2:0] reg_code;
        begin
            case(reg_code)
                3'b100: DecodeRFOutSel = 3'b000;  // R1
                3'b101: DecodeRFOutSel = 3'b001;  // R2
                3'b110: DecodeRFOutSel = 3'b010;  // R3
                3'b111: DecodeRFOutSel = 3'b011;  // R4
                default: DecodeRFOutSel = 3'b000; // Default to R1 if not in RF
            endcase
        end
    endfunction
    
    // Function to determine if a register code refers to ARF
    function IsARFRegister;
        input [2:0] reg_code;
        begin
            IsARFRegister = (reg_code < 3'b100);  // Codes 000-011 are ARF registers
        end
    endfunction
    
    // State counter management with complete instruction cycle termination conditions
    always @(posedge Clock or posedge Reset) begin
        if (Reset) begin
            T_reg <= 5'd1;
        end else begin
            if(T_Reset) begin
                T_reg <= 5'd1;
            end
            // Check for instruction completion or max count
            else if (T_reg == 5'd31 || 
                // Branch instructions
                (T_reg == 5'd2 && (Opcode == 6'h00)) ||                                 // BRA (3 cycles)
                (T_reg == 5'd2 && (Opcode == 6'h01 || Opcode == 6'h02)) ||              // BNE, BEQ (3 cycles)
                
                // Stack operations
                (T_reg == 5'd5 && Opcode == 6'h03) ||                                   // POPL (6 cycles)
                (T_reg == 5'd4 && Opcode == 6'h04) ||                                   // PSHL (5 cycles)
                (T_reg == 5'd7 && Opcode == 6'h05) ||                                   // POPH (8 cycles)
                (T_reg == 5'd8 && Opcode == 6'h06) ||                                   // PSHH (9 cycles)
                (T_reg == 5'd4 && Opcode == 6'h07) ||                                   // CALL (5 cycles)
                (T_reg == 5'd3 && Opcode == 6'h08) ||                                   // RET (4 cycles)
                
                // ALU operations (no memory access)
                (T_reg == 5'd2 && (Opcode >= 6'h09 && Opcode <= 6'h18)) ||              // Single register/ALU ops (3 cycles)
                
                // Memory operations
                (T_reg == 5'd3 && Opcode == 6'h19) ||                                   // MOVL (4 cycles)
                (T_reg == 5'd3 && Opcode == 6'h1A) ||                                   // MOVSH (4 cycles)
                (T_reg == 5'd5 && Opcode == 6'h1B) ||                                   // LDARL (6 cycles)
                (T_reg == 5'd9 && Opcode == 6'h1C) ||                                   // LDARH (10 cycles)
                (T_reg == 5'd3 && Opcode == 6'h1D) ||                                   // STAR (4 cycles)
                (T_reg == 5'd6 && Opcode == 6'h1E) ||                                   // LDAL (7 cycles)
                (T_reg == 5'd10 && Opcode == 6'h1F) ||                                  // LDAH (11 cycles)
                (T_reg == 5'd4 && Opcode == 6'h20) ||                                   // STA (5 cycles)
                (T_reg == 5'd5 && Opcode == 6'h21) ||                                   // LDDRL (6 cycles)
                (T_reg == 5'd9 && Opcode == 6'h22) ||                                   // LDDRH (10 cycles)
                (T_reg == 5'd2 && Opcode == 6'h23) ||                                   // STDR (3 cycles)
                (T_reg == 5'd4 && Opcode == 6'h24)) begin                               // STRIM (5 cycles)
                
                T_reg <= 5'd1;  // Reset state counter
            end else begin
                T_reg <= T_reg + 1;  // Increment state counter
            end
        end
    end
    
    // Main control logic for all instructions
    always @(*) begin
        // Default values for all control signals
        RF_OutASel = 3'b000;
        RF_OutBSel = 3'b000;
        RF_FunSel = 3'b000;
        RF_RegSel = 4'b0000;
        RF_ScrSel = 4'b0000;
        ALU_FunSel = 5'b00000;
        ALU_WF = 1'b0;
        ARF_OutCSel = 2'b00;
        ARF_OutDSel = 2'b00;
        ARF_FunSel = 2'b00;
        ARF_RegSel = 3'b000;
        IR_LH = 1'b0;
        IR_Write = 1'b0;
        Mem_WR = 1'b0;
        Mem_CS = 1'b1;  // Memory disabled by default
        MuxASel = 2'b00;
        MuxBSel = 2'b00;
        MuxCSel = 2'b00;
        MuxDSel = 1'b0;
        DR_E = 1'b0;
        DR_FunSel = 2'b00;
        
        // Decode register selections - update these based on the current instruction
        RF_RSel_Decoded = GetRFRegSel(RegSel);
        DSTREG_Decoded = DecodeDSTREG(DestReg);
        SrcReg1_Decoded = DecodeRFOutSel(SrcReg1);
        SrcReg2_Decoded = DecodeRFOutSel(SrcReg2);
        
        // Fetch Cycle (common for all instructions)
        if (T_reg == 5'd0) begin
            // Fetch LSB of instruction
            IR_LH = 1'b0;                // Load low byte
            IR_Write = 1'b1;             // Enable IR write
            ARF_OutDSel = 2'b00;         // Select PC for memory address
            Mem_CS = 1'b0;               // Enable memory
            Mem_WR = 1'b0;               // Read mode
            
            // PC++
            ARF_RegSel = 3'b100;         // Select PC
            ARF_FunSel = 2'b01;          // Increment
        end
        else if (T_reg == 5'd1) begin
            // Fetch MSB of instruction
            IR_LH = 1'b1;                // Load high byte
            IR_Write = 1'b1;             // Enable IR write
            ARF_OutDSel = 2'b00;         // Select PC for memory address
            Mem_CS = 1'b0;               // Enable memory
            Mem_WR = 1'b0;               // Read mode
            
            // PC++
            ARF_RegSel = 3'b100;         // Select PC
            ARF_FunSel = 2'b01;          // Increment
        end
        else begin
            // Decode and Execute Cycles (T >= 2)
            case(Opcode)
                /*** BRANCH INSTRUCTIONS ***/
                6'h00: begin // BRA (Branch Always)
                    if (T_reg == 5'd2) begin
                        // PC ← VALUE (ADDRESS)
                        ARF_RegSel = 3'b100;       // Select PC
                        ARF_FunSel = 2'b00;        // Load
                        MuxCSel = 2'b11;           // Select IR
                    end
                end
                
                6'h01: begin // BNE (Branch if Not Equal - Z=0)
                    if (T_reg == 5'd2) begin
                        if (Z_Flag == 1'b0) begin  // Z=0
                            // PC ← VALUE (ADDRESS)
                            ARF_RegSel = 3'b100;   // Select PC
                            ARF_FunSel = 2'b00;    // Load
                            MuxCSel = 2'b11;       // Select IR
                        end
                    end
                end
                
                6'h02: begin // BEQ (Branch if Equal - Z=1)
                    if (T_reg == 5'd2) begin
                        if (Z_Flag == 1'b1) begin  // Z=1
                            // PC ← VALUE (ADDRESS)
                            ARF_RegSel = 3'b100;   // Select PC
                            ARF_FunSel = 2'b00;    // Load
                            MuxCSel = 2'b11;       // Select IR
                        end
                    end
                end
                
                //*** STACK OPERATIONS ***/
                6'h03: begin // POPL (Pop Low - 16-bit)
                    if (T_reg == 5'd2) begin
                        // SP ← SP + 1
                        ARF_RegSel = 3'b010;       // Select SP
                        ARF_FunSel = 2'b01;        // Increment
                    end
                    else if (T_reg == 5'd3) begin
                        // Rx ← M[SP] (16-bit) - First read LSB
                        ARF_OutDSel = 2'b01;       // Select SP for memory address
                        Mem_CS = 1'b0;             // Enable memory
                        Mem_WR = 1'b0;             // Read mode
                        MuxASel = 2'b01;           // Select Memory
                        RF_FunSel = 3'b010;        // Load LSB
                        RF_RegSel = RF_RSel_Decoded; // Select Rx based on RegSel
                    end
                    else if (T_reg == 5'd4) begin
                        // SP ← SP + 1
                        ARF_RegSel = 3'b010;       // Select SP
                        ARF_FunSel = 2'b01;        // Increment
                    end
                    else if (T_reg == 5'd5) begin
                        // Rx ← M[SP] (16-bit) - Then read MSB
                        ARF_OutDSel = 2'b01;       // Select SP for memory address
                        Mem_CS = 1'b0;             // Enable memory
                        Mem_WR = 1'b0;             // Read mode
                        MuxASel = 2'b01;           // Select Memory
                        RF_FunSel = 3'b100;        // Load MSB
                        RF_RegSel = RF_RSel_Decoded; // Select Rx based on RegSel
                    end
                end
                
                6'h04: begin // PSHL - 16-bit push from Rx to stack
                    case(T_reg)
                        5'd2: begin
                            // Prepare Rx value for memory
                            RF_OutASel = {1'b1, RF_RSel_Decoded}; // Select Rx
                            MuxCSel = 2'b01;                      // Select high byte (15-8)
                            
                            // Write MSB to memory at SP
                            ARF_OutDSel = 2'b01;  // Select SP as address
                            Mem_CS = 1'b0;        // Enable memory
                            Mem_WR = 1'b1;        // Write mode
                        end
                        
                        5'd3: begin
                            // SP--
                            ARF_RegSel = 3'b010;  // Select SP
                            ARF_FunSel = 2'b00;   // Decrement
                            
                            // Prepare for next write
                            RF_OutASel = {1'b1, RF_RSel_Decoded}; // Select Rx
                            MuxCSel = 2'b00;                      // Select low byte (7-0)
                        end
                        
                        5'd4: begin
                            // Write LSB to memory at SP
                            ARF_OutDSel = 2'b01;  // Select SP as address
                            Mem_CS = 1'b0;        // Enable memory
                            Mem_WR = 1'b1;        // Write mode
                            
                            // SP--
                            ARF_RegSel = 3'b010;  // Select SP
                            ARF_FunSel = 2'b00;   // Decrement
                        end
                    endcase
                end
                
                6'h05: begin // POPH - 32-bit pop from stack to Rx
                    case(T_reg)
                        5'd2: begin
                            // SP++
                            ARF_RegSel = 3'b010;  // Select SP
                            ARF_FunSel = 2'b01;   // Increment
                        end
                        
                        5'd3: begin
                            // Read byte from memory at SP and load into DR
                            DR_E = 1'b1;
                            DR_FunSel = 2'b01;    // Load LSB
                            ARF_OutDSel = 2'b01;  // Select SP as address
                            Mem_CS = 1'b0;        // Enable memory
                            Mem_WR = 1'b0;        // Read mode
                            
                            // SP++
                            ARF_RegSel = 3'b010;  // Select SP
                            ARF_FunSel = 2'b01;   // Increment
                        end
                        
                        5'd4, 5'd5, 5'd6: begin
                            // Read next byte from memory and append to DR
                            DR_E = 1'b1;
                            DR_FunSel = 2'b10;    // Load next byte (shift)
                            ARF_OutDSel = 2'b01;  // Select SP as address
                            Mem_CS = 1'b0;        // Enable memory
                            Mem_WR = 1'b0;        // Read mode
                            
                            // SP++ (only if not last byte)
                            if (T_reg != 5'd6) begin
                                ARF_RegSel = 3'b010;  // Select SP
                                ARF_FunSel = 2'b01;   // Increment
                            end
                        end
                        
                        5'd7: begin
                            // Transfer 32-bit value from DR to Rx
                            MuxASel = 2'b10;                      // Select DR output
                            RF_RegSel = GetRFRegSel(RF_RSel_Decoded); // Select destination register
                            RF_FunSel = 3'b010;                   // Load full 32-bit
                        end
                    endcase
                end
                
                6'h06: begin // PSHH - 32-bit push from Rx to stack
                    case(T_reg)
                        5'd2: begin
                            // Prepare Rx value for memory
                            RF_OutASel = {1'b1, RF_RSel_Decoded}; // Select Rx
                            MuxCSel = 2'b11;                      // MSB (31-24)
                            
                            // Write MSB to memory at SP
                            ARF_OutDSel = 2'b01;  // Select SP as address
                            Mem_CS = 1'b0;        // Enable memory
                            Mem_WR = 1'b1;        // Write mode
                        end
                        
                        5'd3, 5'd5, 5'd7: begin
                            // SP--
                            ARF_RegSel = 3'b010;  // Select SP
                            ARF_FunSel = 2'b00;   // Decrement
                            
                            // Prepare for next write
                            RF_OutASel = {1'b1, RF_RSel_Decoded}; // Select Rx
                            if (T_reg == 5'd3) MuxCSel = 2'b10;       // Select bytes 23-16
                            else if (T_reg == 5'd5) MuxCSel = 2'b01;  // Select bytes 15-8
                            else MuxCSel = 2'b00;                 // Select bytes 7-0
                        end
                        
                        5'd4, 5'd6, 5'd8: begin
                            // Write next byte to memory at SP
                            ARF_OutDSel = 2'b01;  // Select SP as address
                            Mem_CS = 1'b0;        // Enable memory
                            Mem_WR = 1'b1;        // Write mode
                            
                            // SP-- (except after last byte)
                            if (T_reg != 5'd8) begin
                                ARF_RegSel = 3'b010;  // Select SP
                                ARF_FunSel = 2'b00;   // Decrement
                            end
                        end
                    endcase
                end
                
                6'h07: begin // CALL - Call subroutine
                    case(T_reg)
                        5'd2: begin
                            // Push PC to stack
                            MuxCSel = 2'b01;      // Select high byte of PC (15-8)
                            ARF_OutCSel = 2'b00;  // Select PC
                            
                            // Write MSB to memory at SP
                            ARF_OutDSel = 2'b01;  // Select SP as address
                            Mem_CS = 1'b0;        // Enable memory
                            Mem_WR = 1'b1;        // Write mode
                        end
                        
                        5'd3: begin
                            // SP--
                            ARF_RegSel = 3'b010;  // Select SP
                            ARF_FunSel = 2'b00;   // Decrement
                            
                            // Prepare for next write
                            MuxCSel = 2'b00;      // Select low byte of PC (7-0)
                            ARF_OutCSel = 2'b00;  // Select PC
                        end
                        
                        5'd4: begin
                            // Write LSB to memory at SP
                            ARF_OutDSel = 2'b01;  // Select SP as address
                            Mem_CS = 1'b0;        // Enable memory
                            Mem_WR = 1'b1;        // Write mode
                            
                            // SP--
                            ARF_RegSel = 3'b010;  // Select SP
                            ARF_FunSel = 2'b00;   // Decrement
                            
                            // Load target address into PC
                            ARF_RegSel = 3'b100;  // Select PC
                            ARF_FunSel = 2'b10;   // Load
                            MuxBSel = 2'b11;      // Select IR[7:0] as input
                        end
                    endcase
                end
                
                6'h08: begin // RET - Return from subroutine
                    case(T_reg)
                        5'd2: begin
                            // SP++
                            ARF_RegSel = 3'b010;  // Select SP
                            ARF_FunSel = 2'b01;   // Increment
                        end
                        
                        5'd3: begin
                            // Read LSB from memory at SP
                            ARF_OutDSel = 2'b01;  // Select SP as address
                            Mem_CS = 1'b0;        // Enable memory
                            Mem_WR = 1'b0;        // Read mode
                            
                            // Store in PC
                            ARF_RegSel = 3'b100;  // Select PC
                            ARF_FunSel = 2'b10;   // Load
                            MuxBSel = 2'b11;      // Select memory data
                            
                            // SP++
                            ARF_RegSel = 3'b010;  // Select SP
                            ARF_FunSel = 2'b01;   // Increment
                        end
                    endcase
                end
                
                /*** ALU OPERATIONS ***/
                6'h09: begin // INC - Increment
                    if (T_reg == 5'd2) begin
                        // Set up source register
                        if (SrcReg1_Decoded < 3'b100) begin
                            // Source is in ARF (PC, SP, AR)
                            ARF_OutCSel = SrcReg1_Decoded[1:0];  // Select ARF register
                            MuxDSel = 1'b1;                    // Select ARF output
                            ALU_FunSel = 5'b00001;             // INC operation (16-bit)
                        end else begin
                            // Source is in RF (R1-R4)
                            RF_OutASel = SrcReg1_Decoded - 3'b100; // Convert to RF index
                            MuxDSel = 1'b0;                      // Select RF output
                            ALU_FunSel = 5'b10001;               // INC operation (32-bit)
                        end
                        
                        // Set ALU flags
                        ALU_WF = 1'b1;
                        
                        // Set up destination register
                        if (DSTREG_Decoded < 3'b100) begin
                            // Destination is in ARF
                            ARF_RegSel = DSTREG_Decoded;       // Select ARF register
                            ARF_FunSel = 2'b10;                // Load
                            MuxBSel = 2'b00;                   // Select ALU output
                        end else begin
                            // Destination is in RF
                            case(DSTREG_Decoded - 3'b100)
                                3'b000: RF_RegSel = 4'b1000;   // R1
                                3'b001: RF_RegSel = 4'b0100;   // R2
                                3'b010: RF_RegSel = 4'b0010;   // R3
                                3'b011: RF_RegSel = 4'b0001;   // R4
                            endcase
                            RF_FunSel = 3'b010;                // Load
                            MuxASel = 2'b00;                   // Select ALU output
                        end
                    end
                end
                
                6'h0A: begin // DEC - Decrement
                    if (T_reg == 5'd2) begin
                        // Set up source register
                        if (SrcReg1_Decoded < 3'b100) begin
                            // Source is in ARF (PC, SP, AR)
                            ARF_OutCSel = SrcReg1_Decoded[1:0];  // Select ARF register
                            MuxDSel = 1'b1;                    // Select ARF output
                            ALU_FunSel = 5'b00000;             // DEC operation (16-bit)
                        end else begin
                            // Source is in RF (R1-R4)
                            RF_OutASel = SrcReg1_Decoded - 3'b100; // Convert to RF index
                            MuxDSel = 1'b0;                      // Select RF output
                            ALU_FunSel = 5'b10000;               // DEC operation (32-bit)
                        end
                        
                        // Set ALU flags
                        ALU_WF = 1'b1;
                        
                        // Set up destination register
                        if (DSTREG_Decoded < 3'b100) begin
                            // Destination is in ARF
                            ARF_RegSel = DSTREG_Decoded;       // Select ARF register
                            ARF_FunSel = 2'b10;                // Load
                            MuxBSel = 2'b00;                   // Select ALU output
                        end else begin
                            // Destination is in RF
                            case(DSTREG_Decoded - 3'b100)
                                3'b000: RF_RegSel = 4'b1000;   // R1
                                3'b001: RF_RegSel = 4'b0100;   // R2
                                3'b010: RF_RegSel = 4'b0010;   // R3
                                3'b011: RF_RegSel = 4'b0001;   // R4
                            endcase
                            RF_FunSel = 3'b010;                // Load
                            MuxASel = 2'b00;                   // Select ALU output
                        end
                    end
                end
                
                6'h0B: begin // LSL - Logical Shift Left
                    if (T_reg == 5'd2) begin
                        // Set up source register
                        if (SrcReg1_Decoded < 3'b100) begin 
                            // Source is ARF
                            ARF_OutCSel = SrcReg1_Decoded[1:0];
                            MuxDSel = 1'b1;
                            ALU_FunSel = 5'b01011;  // LSL (16-bit)
                        end else begin
                            // Source is RF
                            RF_OutASel = SrcReg1_Decoded - 3'b100;
                            MuxDSel = 1'b0;
                            ALU_FunSel = 5'b11011;  // LSL (32-bit)
                        end
                        
                        // Update flags
                        ALU_WF = 1'b1;
                        
                        // Store result in destination
                        if (DSTREG_Decoded < 3'b100) begin
                            ARF_RegSel = DSTREG_Decoded;
                            ARF_FunSel = 2'b10;  // Load
                            MuxBSel = 2'b00;     // From ALU
                        end else begin
                            case(DSTREG_Decoded - 3'b100)
                                3'b000: RF_RegSel = 4'b1000;  // R1
                                3'b001: RF_RegSel = 4'b0100;  // R2
                                3'b010: RF_RegSel = 4'b0010;  // R3
                                3'b011: RF_RegSel = 4'b0001;  // R4
                            endcase
                            RF_FunSel = 3'b010;  // Load
                            MuxASel = 2'b00;     // From ALU
                        end
                    end
                end

                6'h0C: begin // LSR - Logical Shift Right
                    if (T_reg == 5'd2) begin
                        // Set up source register
                        if (SrcReg1_Decoded < 3'b100) begin 
                            // Source is ARF
                            ARF_OutCSel = SrcReg1_Decoded[1:0];
                            MuxDSel = 1'b1;
                            ALU_FunSel = 5'b01100;  // LSR (16-bit)
                        end else begin
                            // Source is RF
                            RF_OutASel = SrcReg1_Decoded - 3'b100;
                            MuxDSel = 1'b0;
                            ALU_FunSel = 5'b11100;  // LSR (32-bit)
                        end
                        
                        // Update flags
                        ALU_WF = 1'b1;
                        
                        // Store result in destination
                        if (DSTREG_Decoded < 3'b100) begin
                            ARF_RegSel = DSTREG_Decoded;
                            ARF_FunSel = 2'b10;  // Load
                            MuxBSel = 2'b00;     // From ALU
                        end else begin
                            case(DSTREG_Decoded - 3'b100)
                                3'b000: RF_RegSel = 4'b1000;  // R1
                                3'b001: RF_RegSel = 4'b0100;  // R2
                                3'b010: RF_RegSel = 4'b0010;  // R3
                                3'b011: RF_RegSel = 4'b0001;  // R4
                            endcase
                            RF_FunSel = 3'b010;  // Load
                            MuxASel = 2'b00;     // From ALU
                        end
                    end
                end
                
                6'h0D: begin // ASR - Arithmetic Shift Right
                    if (T_reg == 5'd2) begin
                        // Set up source register
                        if (SrcReg1_Decoded < 3'b100) begin 
                            // Source is ARF
                            ARF_OutCSel = SrcReg1_Decoded[1:0];
                            MuxDSel = 1'b1;
                            ALU_FunSel = 5'b01101;  // ASR (16-bit)
                        end else begin
                            // Source is RF
                            RF_OutASel = SrcReg1_Decoded - 3'b100;
                            MuxDSel = 1'b0;
                            ALU_FunSel = 5'b11101;  // ASR (32-bit)
                        end
                        
                        // Update flags
                        ALU_WF = 1'b1;
                        
                        // Store result in destination
                        if (DSTREG_Decoded < 3'b100) begin
                            ARF_RegSel = DSTREG_Decoded;
                            ARF_FunSel = 2'b10;  // Load
                            MuxBSel = 2'b00;     // From ALU
                        end else begin
                            case(DSTREG_Decoded - 3'b100)
                                3'b000: RF_RegSel = 4'b1000;  // R1
                                3'b001: RF_RegSel = 4'b0100;  // R2
                                3'b010: RF_RegSel = 4'b0010;  // R3
                                3'b011: RF_RegSel = 4'b0001;  // R4
                            endcase
                            RF_FunSel = 3'b010;  // Load
                            MuxASel = 2'b00;     // From ALU
                        end
                    end
                end
                
                6'h0E: begin // CSL - Circular Shift Left
                    if (T_reg == 5'd2) begin
                        // Set up source register
                        if (SrcReg1_Decoded < 3'b100) begin 
                            // Source is ARF
                            ARF_OutCSel = SrcReg1_Decoded[1:0];
                            MuxDSel = 1'b1;
                            ALU_FunSel = 5'b01110;  // CSL (16-bit)
                        end else begin
                            // Source is RF
                            RF_OutASel = SrcReg1_Decoded - 3'b100;
                            MuxDSel = 1'b0;
                            ALU_FunSel = 5'b11110;  // CSL (32-bit)
                        end
                        
                        // Update flags
                        ALU_WF = 1'b1;
                        
                        // Store result in destination
                        if (DSTREG_Decoded < 3'b100) begin
                            ARF_RegSel = DSTREG_Decoded;
                            ARF_FunSel = 2'b10;  // Load
                            MuxBSel = 2'b00;     // From ALU
                        end else begin
                            case(DSTREG_Decoded - 3'b100)
                                3'b000: RF_RegSel = 4'b1000;  // R1
                                3'b001: RF_RegSel = 4'b0100;  // R2
                                3'b010: RF_RegSel = 4'b0010;  // R3
                                3'b011: RF_RegSel = 4'b0001;  // R4
                            endcase
                            RF_FunSel = 3'b010;  // Load
                            MuxASel = 2'b00;     // From ALU
                        end
                    end
                end
                
                6'h0F: begin // CSR - Circular Shift Right
                    if (T_reg == 5'd2) begin
                        // Set up source register
                        if (SrcReg1_Decoded < 3'b100) begin 
                            // Source is ARF
                            ARF_OutCSel = SrcReg1_Decoded[1:0];
                            MuxDSel = 1'b1;
                            ALU_FunSel = 5'b01111;  // CSR (16-bit)
                        end else begin
                            // Source is RF
                            RF_OutASel = SrcReg1_Decoded - 3'b100;
                            MuxDSel = 1'b0;
                            ALU_FunSel = 5'b11111;  // CSR (32-bit)
                        end
                        
                        // Update flags
                        ALU_WF = 1'b1;
                        
                        // Store result in destination
                        if (DSTREG_Decoded < 3'b100) begin
                            ARF_RegSel = DSTREG_Decoded;
                            ARF_FunSel = 2'b10;  // Load
                            MuxBSel = 2'b00;     // From ALU
                        end else begin
                            case(DSTREG_Decoded - 3'b100)
                                3'b000: RF_RegSel = 4'b1000;  // R1
                                3'b001: RF_RegSel = 4'b0100;  // R2
                                3'b010: RF_RegSel = 4'b0010;  // R3
                                3'b011: RF_RegSel = 4'b0001;  // R4
                            endcase
                            RF_FunSel = 3'b010;  // Load
                            MuxASel = 2'b00;     // From ALU
                        end
                    end
                end
                
                6'h10: begin // NOT - Bitwise NOT
                    if (T_reg == 5'd2) begin
                        // Set up source register
                        if (SrcReg1_Decoded < 3'b100) begin 
                            // Source is ARF
                            ARF_OutCSel = SrcReg1_Decoded[1:0];
                            MuxDSel = 1'b1;
                            ALU_FunSel = 5'b00010;  // NOT (16-bit)
                        end else begin
                            // Source is RF
                            RF_OutASel = SrcReg1_Decoded - 3'b100;
                            MuxDSel = 1'b0;
                            ALU_FunSel = 5'b10010;  // NOT (32-bit)
                        end
                        
                        // Update flags
                        ALU_WF = 1'b1;
                        
                        // Store result in destination
                        if (DSTREG_Decoded < 3'b100) begin
                            ARF_RegSel = DSTREG_Decoded;
                            ARF_FunSel = 2'b10;  // Load
                            MuxBSel = 2'b00;     // From ALU
                        end else begin
                            case(DSTREG_Decoded - 3'b100)
                                3'b000: RF_RegSel = 4'b1000;  // R1
                                3'b001: RF_RegSel = 4'b0100;  // R2
                                3'b010: RF_RegSel = 4'b0010;  // R3
                                3'b011: RF_RegSel = 4'b0001;  // R4
                            endcase
                            RF_FunSel = 3'b010;  // Load
                            MuxASel = 2'b00;     // From ALU
                        end
                    end
                end
                
                6'h11: begin // AND - Bitwise AND
                    if (T_reg == 5'd2) begin
                        // Set up source registers
                        if (SrcReg1_Decoded < 3'b100) begin 
                            // Source1 is ARF
                            ARF_OutCSel = SrcReg1_Decoded[1:0];
                            MuxDSel = 1'b1;
                        end else begin
                            // Source1 is RF
                            RF_OutASel = SrcReg1_Decoded - 3'b100;
                            MuxDSel = 1'b0;
                        end
                        
                        // Source2 is RF (must be loaded to scratch register)
                        RF_OutBSel = SrcReg2_Decoded;
                        
                        // Select AND operation
                        ALU_FunSel = (SrcReg1_Decoded < 3'b100) ? 5'b00111 : 5'b10111;  // AND (16/32-bit)
                        
                        // Update flags
                        ALU_WF = 1'b1;
                        
                        // Store result in destination
                        if (DSTREG_Decoded < 3'b100) begin
                            ARF_RegSel = DSTREG_Decoded;
                            ARF_FunSel = 2'b10;  // Load
                            MuxBSel = 2'b00;     // From ALU
                        end else begin
                            case(DSTREG_Decoded - 3'b100)
                                3'b000: RF_RegSel = 4'b1000;  // R1
                                3'b001: RF_RegSel = 4'b0100;  // R2
                                3'b010: RF_RegSel = 4'b0010;  // R3
                                3'b011: RF_RegSel = 4'b0001;  // R4
                            endcase
                            RF_FunSel = 3'b010;  // Load
                            MuxASel = 2'b00;     // From ALU
                        end
                    end
                end
                
                6'h12: begin // ORR - Bitwise OR
                    if (T_reg == 5'd2) begin
                        // Set up source registers
                        if (SrcReg1_Decoded < 3'b100) begin 
                            // Source1 is ARF
                            ARF_OutCSel = SrcReg1_Decoded[1:0];
                            MuxDSel = 1'b1;
                        end else begin
                            // Source1 is RF
                            RF_OutASel = SrcReg1_Decoded - 3'b100;
                            MuxDSel = 1'b0;
                        end
                        
                        // Source2 is RF
                        RF_OutBSel = SrcReg2_Decoded;
                        
                        // Select OR operation
                        ALU_FunSel = (SrcReg1_Decoded < 3'b100) ? 5'b01000 : 5'b11000;  // OR (16/32-bit)
                        
                        // Update flags
                        ALU_WF = 1'b1;
                        
                        // Store result in destination
                        if (DSTREG_Decoded < 3'b100) begin
                            ARF_RegSel = DSTREG_Decoded;
                            ARF_FunSel = 2'b10;  // Load
                            MuxBSel = 2'b00;     // From ALU
                        end else begin
                            case(DSTREG_Decoded - 3'b100)
                                3'b000: RF_RegSel = 4'b1000;  // R1
                                3'b001: RF_RegSel = 4'b0100;  // R2
                                3'b010: RF_RegSel = 4'b0010;  // R3
                                3'b011: RF_RegSel = 4'b0001;  // R4
                            endcase
                            RF_FunSel = 3'b010;  // Load
                            MuxASel = 2'b00;     // From ALU
                        end
                    end
                end
                
                6'h13: begin // XOR - Bitwise XOR
                    if (T_reg == 5'd2) begin
                        // Set up source registers
                        if (SrcReg1_Decoded < 3'b100) begin 
                            // Source1 is ARF
                            ARF_OutCSel = SrcReg1_Decoded[1:0];
                            MuxDSel = 1'b1;
                        end else begin
                            // Source1 is RF
                            RF_OutASel = SrcReg1_Decoded - 3'b100;
                            MuxDSel = 1'b0;
                        end
                        
                        // Source2 is RF
                        RF_OutBSel = SrcReg2_Decoded;
                        
                        // Select XOR operation
                        ALU_FunSel = (SrcReg1_Decoded < 3'b100) ? 5'b01001 : 5'b11001;  // XOR (16/32-bit)
                        
                        // Update flags
                        ALU_WF = 1'b1;
                        
                        // Store result in destination
                        if (DSTREG_Decoded < 3'b100) begin
                            ARF_RegSel = DSTREG_Decoded;
                            ARF_FunSel = 2'b10;  // Load
                            MuxBSel = 2'b00;     // From ALU
                        end else begin
                            case(DSTREG_Decoded - 3'b100)
                                3'b000: RF_RegSel = 4'b1000;  // R1
                                3'b001: RF_RegSel = 4'b0100;  // R2
                                3'b010: RF_RegSel = 4'b0010;  // R3
                                3'b011: RF_RegSel = 4'b0001;  // R4
                            endcase
                            RF_FunSel = 3'b010;  // Load
                            MuxASel = 2'b00;     // From ALU
                        end
                    end
                end
                
                6'h14: begin // NAND - Bitwise NAND
                    if (T_reg == 5'd2) begin
                        // Set up source registers
                        if (SrcReg1_Decoded < 3'b100) begin 
                            // Source1 is ARF
                            ARF_OutCSel = SrcReg1_Decoded[1:0];
                            MuxDSel = 1'b1;
                        end else begin
                            // Source1 is RF
                            RF_OutASel = SrcReg1_Decoded - 3'b100;
                            MuxDSel = 1'b0;
                        end
                        
                        // Source2 is RF
                        RF_OutBSel = SrcReg2_Decoded;
                        
                        // Select NAND operation
                        ALU_FunSel = (SrcReg1_Decoded < 3'b100) ? 5'b01010 : 5'b11010;  // NAND (16/32-bit)
                        
                        // Update flags
                        ALU_WF = 1'b1;
                        
                        // Store result in destination
                        if (DSTREG_Decoded < 3'b100) begin
                            ARF_RegSel = DSTREG_Decoded;
                            ARF_FunSel = 2'b10;  // Load
                            MuxBSel = 2'b00;     // From ALU
                        end else begin
                            case(DSTREG_Decoded - 3'b100)
                                3'b000: RF_RegSel = 4'b1000;  // R1
                                3'b001: RF_RegSel = 4'b0100;  // R2
                                3'b010: RF_RegSel = 4'b0010;  // R3
                                3'b011: RF_RegSel = 4'b0001;  // R4
                            endcase
                            RF_FunSel = 3'b010;  // Load
                            MuxASel = 2'b00;     // From ALU
                        end
                    end
                end
                
                6'h15: begin // ADD - Addition
                    if (T_reg == 5'd2) begin
                        // Set up source registers
                        if (SrcReg1_Decoded < 3'b100) begin 
                            // Source1 is ARF
                            ARF_OutCSel = SrcReg1_Decoded[1:0];
                            MuxDSel = 1'b1;
                        end else begin
                            // Source1 is RF
                            RF_OutASel = SrcReg1_Decoded - 3'b100;
                            MuxDSel = 1'b0;
                        end
                        
                        // Source2 is RF
                        RF_OutBSel = SrcReg2_Decoded;
                        
                        // Select ADD operation
                        ALU_FunSel = (SrcReg1_Decoded < 3'b100) ? 5'b00100 : 5'b10100;  // ADD (16/32-bit)
                        
                        // Update flags
                        ALU_WF = 1'b1;
                        
                        // Store result in destination
                        if (DSTREG_Decoded < 3'b100) begin
                            ARF_RegSel = DSTREG_Decoded;
                            ARF_FunSel = 2'b10;  // Load
                            MuxBSel = 2'b00;     // From ALU
                        end else begin
                            case(DSTREG_Decoded - 3'b100)
                                3'b000: RF_RegSel = 4'b1000;  // R1
                                3'b001: RF_RegSel = 4'b0100;  // R2
                                3'b010: RF_RegSel = 4'b0010;  // R3
                                3'b011: RF_RegSel = 4'b0001;  // R4
                            endcase
                            RF_FunSel = 3'b010;  // Load
                            MuxASel = 2'b00;     // From ALU
                        end
                    end
                end
                
                6'h16: begin // ADC - Add with Carry
                    if (T_reg == 5'd2) begin
                        // Set up source registers
                        if (SrcReg1_Decoded < 3'b100) begin 
                            // Source1 is ARF
                            ARF_OutCSel = SrcReg1_Decoded[1:0];
                            MuxDSel = 1'b1;
                        end else begin
                            // Source1 is RF
                            RF_OutASel = SrcReg1_Decoded - 3'b100;
                            MuxDSel = 1'b0;
                        end
                        
                        // Source2 is RF
                        RF_OutBSel = SrcReg2_Decoded;
                        
                        // Select ADC operation
                        ALU_FunSel = (SrcReg1_Decoded < 3'b100) ? 5'b00101 : 5'b10101;  // ADC (16/32-bit)
                        
                        // Update flags
                        ALU_WF = 1'b1;
                        
                        // Store result in destination
                        if (DSTREG_Decoded < 3'b100) begin
                            ARF_RegSel = DSTREG_Decoded;
                            ARF_FunSel = 2'b10;  // Load
                            MuxBSel = 2'b00;     // From ALU
                        end else begin
                            case(DSTREG_Decoded - 3'b100)
                                3'b000: RF_RegSel = 4'b1000;  // R1
                                3'b001: RF_RegSel = 4'b0100;  // R2
                                3'b010: RF_RegSel = 4'b0010;  // R3
                                3'b011: RF_RegSel = 4'b0001;  // R4
                            endcase
                            RF_FunSel = 3'b010;  // Load
                            MuxASel = 2'b00;     // From ALU
                        end
                    end
                end
                
                6'h17: begin // SUB - Subtraction
                    if (T_reg == 5'd2) begin
                        // Set up source registers
                        if (SrcReg1_Decoded < 3'b100) begin 
                            // Source1 is ARF
                            ARF_OutCSel = SrcReg1_Decoded[1:0];
                            MuxDSel = 1'b1;
                        end else begin
                            // Source1 is RF
                            RF_OutASel = SrcReg1_Decoded - 3'b100;
                            MuxDSel = 1'b0;
                        end
                        
                        // Source2 is RF
                        RF_OutBSel = SrcReg2_Decoded;
                        
                        // Select SUB operation
                        ALU_FunSel = (SrcReg1_Decoded < 3'b100) ? 5'b00110 : 5'b10110;  // SUB (16/32-bit)
                        
                        // Update flags
                        ALU_WF = 1'b1;
                        
                        // Store result in destination
                        if (DSTREG_Decoded < 3'b100) begin
                            ARF_RegSel = DSTREG_Decoded;
                            ARF_FunSel = 2'b10;  // Load
                            MuxBSel = 2'b00;     // From ALU
                        end else begin
                            case(DSTREG_Decoded - 3'b100)
                                3'b000: RF_RegSel = 4'b1000;  // R1
                                3'b001: RF_RegSel = 4'b0100;  // R2
                                3'b010: RF_RegSel = 4'b0010;  // R3
                                3'b011: RF_RegSel = 4'b0001;  // R4
                            endcase
                            RF_FunSel = 3'b010;  // Load
                            MuxASel = 2'b00;     // From ALU
                        end
                    end
                end
                
                6'h18: begin
                    // Select source register based on SrcReg1
                    if (SrcReg1 < 3'b100) begin  // ARF register (PC, SP, AR)
                        ARF_OutCSel = SrcReg1[1:0];  // Select ARF register
                        MuxASel = 2'b01;           // ARF_OutC to MuxA
                        ALU_FunSel = 5'b10000;     // ALU Pass A
                    end else begin  // RF register (R1, R2, R3, R4)
                        RF_OutASel = SrcReg1;        // Select RF register
                        MuxASel = 2'b00;           // RF_OutA to MuxA
                        ALU_FunSel = 5'b10000;     // ALU Pass A
                    end
                    
                    // Destination register based on DestReg
                    if (DestReg < 3'b100) begin  // ARF register (PC, SP, AR)
                        ARF_RegSel = (DestReg == 3'b000) ? 3'b100 :
                                        (DestReg == 3'b001) ? 3'b010 :
                                        3'b001;        // Select ARF register
                        ARF_FunSel = 2'b10;         // Load
                        MuxDSel = 1'b0;             // ALU_Out to ARF
                    end else begin  // RF register (R1, R2, R3, R4)
                        RF_RegSel = (DestReg == 3'b100) ? 4'b1000 :
                                    (DestReg == 3'b101) ? 4'b0100 :
                                    (DestReg == 3'b110) ? 4'b0010 :
                                    4'b0001;         // Select RF register
                        RF_FunSel = 2'b10;           // Load
                    end
                end
                
                // 0x19: MOVL Rx[7:0] ← IMMEDIATE (8-bit)
                6'h19: begin
                    MuxASel = 2'b11;            // IR[7:0] to MuxA
                    ALU_FunSel = 5'b10000;      // ALU Pass A
                    
                    // Select destination register based on RegSel
                    case (RegSel)
                        2'b00: RF_RegSel = 4'b1000;  // R1
                        2'b01: RF_RegSel = 4'b0100;  // R2
                        2'b10: RF_RegSel = 4'b0010;  // R3
                        2'b11: RF_RegSel = 4'b0001;  // R4
                    endcase
                    
                    RF_FunSel = 2'b10;          // Load to Register File
                end
                
                // 0x1A: MOVSH Rx[31-8] ← Rx[23-0] (8-bit Left Shift), Rx[7-0] ← IMMEDIATE (8-bit)
                6'h1A: begin
                    // Select register to shift and load based on RegSel
                    case (RegSel)
                        2'b00: RF_RegSel = 4'b1000;  // R1
                        2'b01: RF_RegSel = 4'b0100;  // R2
                        2'b10: RF_RegSel = 4'b0010;  // R3
                        2'b11: RF_RegSel = 4'b0001;  // R4
                    endcase
                    
                    // Load immediate to lower 8 bits and shift higher bits
                    RF_FunSel = 2'b10;          // Function is Load 
                    MuxASel = 2'b11;            // IR[7:0] to MuxA
                    ALU_FunSel = 5'b10000;      // ALU Pass A
                    RF_FunSel = 2'b11;          // Special operation for this instruction
                end
                
                // 0x1B: LDARL DestReg ← M[AR] (16-bit)
                6'h1B: begin
                    // Set AR as address
                    ARF_OutCSel = 2'b10;        // AR to OutC
                    MuxASel = 2'b01;            // ARF_OutC to MuxA
                    ALU_FunSel = 5'b10000;      // ALU Pass A
                    
                    // Read from memory
                    Mem_CS = 1'b1;              // Memory chip select
                    Mem_WR = 8'h01;             // Memory read
                    
                    // Load into DR register first (LSB)
                    DR_FunSel = 2'b01;          // Load to DR
                    DR_E = 1'b1;           // Enable DR
                    
                    // Additional states needed to load MSB and transfer to destination
                    // This will be handled in T3 state
                end
                
                // 0x1C: LDARH DestReg ← M[AR] (32-bit)
                6'h1C: begin
                    // Set AR as address
                    ARF_OutCSel = 2'b10;        // AR to OutC
                    MuxASel = 2'b01;            // ARF_OutC to MuxA
                    ALU_FunSel = 5'b10000;      // ALU Pass A
                    
                    // Read from memory
                    Mem_CS = 1'b1;              // Memory chip select
                    Mem_WR = 8'h01;             // Memory read
                    
                    // Load into DR register first (byte 0)
                    DR_FunSel = 2'b01;          // Load to DR
                    DR_E = 1'b1;           // Enable DR
                    
                    // Additional states needed to load bytes 1-3
                    // This will be handled in T3-T5 states
                end
                
                // 0x1D: STAR M[AR] ← SrcReg1
                6'h1D: begin
                    // Select source register
                    if (SrcReg1 < 3'b100) begin  // ARF register (PC, SP, AR)
                        ARF_OutCSel = SrcReg1[1:0];  // Select ARF register
                        MuxASel = 2'b01;           // ARF_OutC to MuxA
                    end else begin  // RF register (R1, R2, R3, R4)
                        RF_OutASel = SrcReg1;        // Select RF register
                        MuxASel = 2'b00;           // RF_OutA to MuxA
                    end
                    
                    // Set address to AR
                    ARF_OutCSel = 2'b10;        // AR to OutC
                    
                    // ALU just passes through the data
                    ALU_FunSel = 5'b10000;      // ALU Pass A
                    
                    // Write to memory
                    Mem_CS = 1'b1;              // Memory chip select
                    Mem_WR = 8'h02;             // Memory write
                    
                    // If 32-bit register, need to write higher bytes in T3-T5
                end
                
                // 0x1E: LDAL Rx ← M[ADDRESS] (16-bit)
                6'h1E: begin
                    // Set AR to ADDRESS
                    MuxASel = 2'b11;            // IR[7:0] to MuxA (ADDRESS)
                    ALU_FunSel = 5'b10000;      // ALU Pass A
                    MuxDSel = 1'b0;             // ALU_Out to MuxD
                    ARF_RegSel = 3'b001;        // Select AR
                    ARF_FunSel = 2'b10;         // Load ADDRESS to AR
                    
                    // Memory read will happen in T3
                end
                
                // 0x1F: LDAH Rx ← M[ADDRESS] (32-bit)
                6'h1F: begin
                    // Set AR to ADDRESS
                    MuxASel = 2'b11;            // IR[7:0] to MuxA (ADDRESS)
                    ALU_FunSel = 5'b10000;      // ALU Pass A
                    MuxDSel = 1'b0;             // ALU_Out to MuxD
                    ARF_RegSel = 3'b001;        // Select AR
                    ARF_FunSel = 2'b10;         // Load ADDRESS to AR
                    
                    // Memory read will happen in T3-T6
                end
                
                // 0x20: STA M[ADDRESS] ← Rx
                6'h20: begin
                    // Set AR to ADDRESS
                    MuxASel = 2'b11;            // IR[7:0] to MuxA (ADDRESS)
                    ALU_FunSel = 5'b10000;      // ALU Pass A
                    MuxDSel = 1'b0;             // ALU_Out to MuxD
                    ARF_RegSel = 3'b001;        // Select AR
                    ARF_FunSel = 2'b10;         // Load ADDRESS to AR
                    
                    // Select source register based on RegSel
                    case (RegSel)
                        2'b00: RF_OutASel = 3'b100;  // R1
                        2'b01: RF_OutASel = 3'b101;  // R2
                        2'b10: RF_OutASel = 3'b110;  // R3
                        2'b11: RF_OutASel = 3'b111;  // R4
                    endcase
                    
                    // Memory write will happen in T3
                end
                
                // 0x21: LDDRL DR ← M[AR] (16-bit)
                6'h21: begin
                    // Set AR as address
                    ARF_OutCSel = 2'b10;        // AR to OutC
                    
                    // Read from memory
                    Mem_CS = 1'b1;              // Memory chip select
                    Mem_WR = 8'h01;             // Memory read
                    
                    // Load into DR register (LSB)
                    DR_FunSel = 2'b01;          // Load to DR
                    DR_E = 1'b1;           // Enable DR
                    
                    // T3 will handle loading MSB
                end
                
                // 0x22: LDDRH DR ← M[AR] (32-bit)
                6'h22: begin
                    // Set AR as address
                    ARF_OutCSel = 2'b10;        // AR to OutC
                    
                    // Read from memory
                    Mem_CS = 1'b1;              // Memory chip select
                    Mem_WR = 8'h01;             // Memory read
                    
                    // Load into DR register (byte 0)
                    DR_FunSel = 2'b01;          // Load to DR
                    DR_E = 1'b1;           // Enable DR
                    
                    // T3-T5 will handle loading bytes 1-3
                end
                
                // 0x23: STDR DestReg ← DR
                6'h23: begin
                    // Move DR to ALU
                    MuxASel = 2'b10;            // DR to MuxA
                    ALU_FunSel = 5'b10000;      // ALU Pass A
                    
                    // Select destination register based on DestReg
                    if (DestReg < 3'b100) begin  // ARF register (PC, SP, AR)
                        ARF_RegSel = (DestReg == 3'b000) ? 3'b100 :
                                        (DestReg == 3'b001) ? 3'b010 :
                                        3'b001;        // Select ARF register
                        ARF_FunSel = 2'b10;         // Load
                        MuxDSel = 1'b0;             // ALU_Out to ARF
                    end else begin  // RF register (R1, R2, R3, R4)
                        RF_RegSel = (DestReg == 3'b100) ? 4'b1000 :
                                    (DestReg == 3'b101) ? 4'b0100 :
                                    (DestReg == 3'b110) ? 4'b0010 :
                                    4'b0001;         // Select RF register
                        RF_FunSel = 2'b10;           // Load
                    end
                end
                
                // 0x24: STRIM M[AR+OFFSET] ← Rx (AR is 16-bit register)
                6'h24: begin
                    // Calculate AR + OFFSET
                    ARF_OutCSel = 2'b10;        // AR to OutC
                    MuxASel = 2'b01;            // ARF_OutC to MuxA
                    MuxBSel = 2'b11;            // IR[7:0] to MuxB (OFFSET)
                    ALU_FunSel = 5'b10100;      // ALU A + B
                    
                    // Store the result in a temporary register or directly use
                    // This depends on the implementation strategy
                    
                    // Select source register based on RegSel for data to store
                    case (RegSel)
                        2'b00: RF_OutASel = 3'b100;  // R1
                        2'b01: RF_OutASel = 3'b101;  // R2
                        2'b10: RF_OutASel = 3'b110;  // R3
                        2'b11: RF_OutASel = 3'b111;  // R4
                    endcase
                    
                    // Memory write will happen in T3 after address calculation
                end

                default: begin
                    // Default case for unimplemented Opcodes
                    // No operations, continue to next instruction
                end
            endcase
        end
    end
    
endmodule