`timescale 1ns / 1ps

module ArithmeticLogicUnit(
    input wire Clock,
    input wire [31:0] A,
    input wire [31:0] B,
    input wire [4:0] FunSel,
    output reg [3:0] FlagsOut,
    output reg [31:0] ALUOut,
    input wire WF
);

    reg [32:0] temp;
    reg [31:0] result;
    reg next_carry;
    reg next_overflow;
    reg next_zero;
    reg next_negative;

    always @(*) begin
        temp = 33'b0;
        result = 32'b0;
        next_carry = 1'b0;
        next_overflow = 1'b0;

        case (FunSel)
            5'b00000: result = {16'b0, A[15:0]};
            5'b00001: result = {16'b0, B[15:0]};
            5'b00010: result = {16'b0, ~A[15:0]};
            5'b00011: result = {16'b0, ~B[15:0]};

            5'b00100: begin
                temp = {1'b0, A[15:0]} + {1'b0, B[15:0]};
                result = {16'b0, temp[15:0]};
                next_carry = temp[16];
                next_overflow = (~A[15] & ~B[15] & result[15]) | (A[15] & B[15] & ~result[15]);
            end

            5'b00101: begin
                temp = {1'b0, A[15:0]} + {1'b0, B[15:0]} + FlagsOut[2];
                result = {16'b0, temp[15:0]};
                next_carry = temp[16];
                next_overflow = (~A[15] & ~B[15] & result[15]) | (A[15] & B[15] & ~result[15]);
            end

            5'b00110: begin
                temp = {1'b0, A[15:0]} + {1'b0, ~B[15:0]} + 1'b1;
                result = {16'b0, temp[15:0]};
                next_carry = temp[16];
                next_overflow = (~A[15] & B[15] & result[15]) | (A[15] & ~B[15] & ~result[15]);
            end

            5'b00111: result = {16'b0, A[15:0] & B[15:0]};
            5'b01000: result = {16'b0, A[15:0] | B[15:0]};
            5'b01001: result = {16'b0, A[15:0] ^ B[15:0]};
            5'b01010: result = {16'b0, ~(A[15:0] & B[15:0])};

            5'b01011: begin result = {16'b0, A[14:0], 1'b0}; next_carry = A[15]; end
            5'b01100: begin result = {16'b0, 1'b0, A[15:1]}; next_carry = A[0]; end
            5'b01101: begin result = {16'b0, A[15], A[15:1]}; next_carry = A[0]; end
            5'b01110: begin result = {16'b0, FlagsOut[2], A[15:1]}; next_carry = A[0]; end
            5'b01111: begin result = {16'b0, A[14:0], FlagsOut[2]}; next_carry = A[15]; end

            5'b10000: result = A;
            5'b10001: result = B;
            5'b10010: result = ~A;
            5'b10011: result = ~B;

            5'b10100: begin
                temp = {1'b0, A} + {1'b0, B};
                result = temp[31:0];
                next_carry = temp[32];
                next_overflow = (~A[31] & ~B[31] & result[31]) | (A[31] & B[31] & ~result[31]);
            end

            5'b10101: begin
                temp = {1'b0, A} + {1'b0, B} + FlagsOut[2];
                result = temp[31:0];
                next_carry = temp[32];
                next_overflow = (~A[31] & ~B[31] & result[31]) | (A[31] & B[31] & ~result[31]);
            end

            5'b10110: begin
                temp = {1'b0, A} + {1'b0, ~B} + 1'b1;
                result = temp[31:0];
                next_carry = temp[32];
                next_overflow = (~A[31] & B[31] & result[31]) | (A[31] & ~B[31] & ~result[31]);
            end

            5'b10111: result = A & B;
            5'b11000: result = A | B;
            5'b11001: result = A ^ B;
            5'b11010: result = ~(A & B);

            5'b11011: begin result = {A[30:0], 1'b0}; next_carry = A[31]; end
            5'b11100: begin result = {1'b0, A[31:1]}; next_carry = A[0]; end
            5'b11101: begin result = {A[31], A[31:1]}; next_carry = A[0]; end
            5'b11110: begin result = {FlagsOut[2], A[31:1]}; next_carry = A[0]; end
            5'b11111: begin result = {A[30:0], FlagsOut[2]}; next_carry = A[31]; end

            default: result = 32'b0;
        endcase


        ALUOut = result;

        next_zero = (result == 32'b0);
        next_negative = FunSel[4] ? result[31] : result[15];
    end

    always @(posedge Clock) begin
        if (WF) begin
            FlagsOut[3] <= next_zero;
            FlagsOut[2] <= next_carry;
            FlagsOut[1] <= next_negative;
            FlagsOut[0] <= next_overflow;
        end
    end

endmodule