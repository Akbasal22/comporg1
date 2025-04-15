`timescale 1ns / 1ps

module RegisterFile(
    input wire Clock,
    input wire [31:0] I,
    input wire [3:0] RegSel,
    input wire [3:0] ScrSel,
    input wire [2:0] FunSel,
    input wire [2:0] OutASel,
    input wire [2:0] OutBSel,
    output reg [31:0] OutA,
    output reg [31:0] OutB
);

    // === Internal wires ===
    wire [31:0] R1_out, R2_out, R3_out, R4_out;
    wire [31:0] S1_out, S2_out, S3_out, S4_out;

     // === Output MUX ===
    always @(*) begin
            // Normal operation for other cases
            case (OutASel)
                3'b000: OutA = R1_out;
                3'b001: OutA = R2_out;
                3'b010: OutA = R3_out;
                3'b011: OutA = R4_out;
                3'b100: OutA = S1_out;
                3'b101: OutA = S2_out;
                3'b110: OutA = S3_out;
                3'b111: OutA = S4_out;
                default: OutA = 32'b0;
            endcase
            
            case (OutBSel)
                3'b000: OutB = R1_out;
                3'b001: OutB = R2_out;
                3'b010: OutB = R3_out;
                3'b011: OutB = R4_out;
                3'b100: OutB = S1_out;
                3'b101: OutB = S2_out;
                3'b110: OutB = S3_out;
                3'b111: OutB = S4_out;
                default: OutB=32'b0;
            endcase
        end


    // === Named register instances (so testbench can access) ===
    Register32bit R1 (.I(I), .Clock(Clock), .E(RegSel[3]), .FunSel(FunSel), .Q(R1_out));
    Register32bit R2 (.I(I), .Clock(Clock), .E(RegSel[2]), .FunSel(FunSel), .Q(R2_out));
    Register32bit R3 (.I(I), .Clock(Clock), .E(RegSel[1]), .FunSel(FunSel), .Q(R3_out));
    Register32bit R4 (.I(I), .Clock(Clock), .E(RegSel[0]), .FunSel(FunSel), .Q(R4_out));

    Register32bit S1 (.I(I), .Clock(Clock), .E(ScrSel[3]), .FunSel(FunSel), .Q(S1_out));
    Register32bit S2 (.I(I), .Clock(Clock), .E(ScrSel[2]), .FunSel(FunSel), .Q(S2_out));
    Register32bit S3 (.I(I), .Clock(Clock), .E(ScrSel[1]), .FunSel(FunSel), .Q(S3_out));
    Register32bit S4 (.I(I), .Clock(Clock), .E(ScrSel[0]), .FunSel(FunSel), .Q(S4_out));

   

endmodule