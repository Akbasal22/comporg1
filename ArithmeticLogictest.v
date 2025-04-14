`timescale 1ns / 1ps
module ArithmeticLogicUnit( // This function might not be what is expected
        input wire Clock,
        input wire [31:0] A,  //in1
        input wire [31:0] B,  //in2     
        input wire [4:0] FunSel, 
        output reg [3:0] FlagsOut, //Z=3, C=2= N=1, O=0     
        output reg [31:0] ALUOut, //output
        input wire WF
    );

    reg [32:0] temp; // for the 1-bit carry
    reg carry, overflow;

    always @(posedge Clock) begin
        carry=0;
        overflow=0;
        case (FunSel)
            //16 bit operations
            5'b00000: begin ALUOut <= {16'b0, A[15:0]};end
            5'b00001: begin ALUOut <= {16'b0, B[15:0]}; end
            5'b00010: begin ALUOut <= {16'b0, ~A[15:0]}; end
            5'b00011: begin ALUOut <= {16'b0, ~B[15:0]}; end
            5'b00100: begin // A+B
                temp = {1'b0, A[15:0]} + {1'b0, B[15:0]}; //add 17th bit -> we will check if there is carry
                ALUOut <= {16'b0, temp[15:0]};
                carry = temp[16]; //if 16th bit is 1, there is carry
                overflow = (~A[15] & ~B[15] & temp[15] ) | (A[15] & B[15] & ~temp[15]); //pos+pos=neg OR neg+neg=pos
            end 
            5'b00101: begin                 
                temp = {1'b0, A[15:0]} + {1'b0, B[15:0]} + FlagsOut[2]; //add 17th bit -> we will check if there is carry
                ALUOut <= {16'b0, temp[15:0]};
                carry = temp[16]; //if 16th bit is 1, there is carry
                overflow = (~A[15] & ~B[15] & temp[15] ) | (A[15] & B[15] & ~temp[15]); //pos+pos=neg OR neg+neg=pos; end 
            end
            5'b00110: begin                 
                temp = {1'b0, A[15:0]} + {1'b0, ~B[15:0]} + 1; //add 17th bit -> we will check if there is carry
                ALUOut <= {16'b0, temp[15:0]};
                carry = temp[16]; //if 16th bit is 1, there is carry
                overflow = (A[15] & ~B[15] & ~temp[15] ) | (~A[15] & B[15] & temp[15]); //different pos-neg=neg OR neg-pos=pos
            end
            5'b00111: begin ALUOut <= {16'b0, A[15:0] & B[15:0]}; end //setting the first 16 bits to 0 for robustness 
            5'b01000: begin ALUOut <= {16'b0, A[15:0] | B[15:0]}; end
            5'b01001: begin ALUOut <= {16'b0, A[15:0] ^ B[15:0]}; end
            5'b01010: begin ALUOut <= {16'b0, ~(A[15:0] & B[15:0])}; end
            5'b01011: begin ALUOut <= {16'b0, A[14:0] , 1'b0}; carry=A[15]; end //again, setting the first 16bits to 0
            5'b01100: begin ALUOut <= {16'b0, 1'b0, A[15:1]}; carry=A[0]; end
            5'b01101: begin ALUOut <= {16'b0, A[15], A[15:1]}; end
            5'b01110: begin ALUOut <= {16'FlagsOut[2], A[15:1]}; carry = A[0]; end
            5'b01111: begin ALUOut <= {16'b0, A[14:0], FlagsOut[2]}; carry = A[15]; end

            //32 bit operations
            5'b10000: begin ALUOut <= A; end
            5'b10001: begin ALUOut <= B; end
            5'b10010: begin ALUOut <= ~A; end
            5'b10011: begin ALUOut <= ~B; end
            5'b10100: begin //A+B
                temp = {1'b0, A} + {1'b0, B}; //33th bit -> to check carry
                ALUOut <= temp[31:0];
                carry = temp[32];
                overflow = (~A[31] & ~B[31] & temp[31]) | (A[31] & B[31] & ~temp[31]);
            end
            5'b10101: begin                 
                temp = {1'b0, A} + {1'b0, B} + FlagsOut[2]; //33th bit -> to check carry
                ALUOut <= temp[31:0];
                carry = temp[32];
                overflow = (~A[31] & ~B[31] & temp[31]) | (A[31] & B[31] & ~temp[31]);
            end
            5'b10110: begin                 
                temp = {1'b0, A} + {1'b0, ~B} + 1 ;//33th bit -> to check carry
                ALUOut <= temp[31:0];
                carry = temp[32];
                overflow = (A[31] & ~B[31] & ~temp[31]) | (~A[31] & B[31] & temp[31]);
            end
            5'b10111: begin ALUOut <= A & B; end
            5'b11000: begin ALUOut <= A | B; end
            5'b11001: begin ALUOut <= A ^ B; end
            5'b11010: begin ALUOut <= ~(A & B); end
            5'b11011: begin ALUOut <= {A[30:0] , 1'b0}; FlagsOut[2]=A[31]; end
            5'b11100: begin ALUOut <= {1'b0, A[31:1]}; FlagsOut[2]=A[0]; end
            5'b11101: begin ALUOut <= {A[31], A[31:1]}; end
            5'b11110: begin ALUOut <= {FlagsOut[2], A[31:1]}; carry = A[0]; end
            5'b11111: begin ALUOut <= {A[30:0], FlagsOut[2]}; carry = A[31]; end
            default: ALUOut <= 32'hDEADBEEF; // for debugging
        endcase

        //flag calcs
        FlagsOut[3] <= (ALUOut == 0);
        FlagsOut[2] <= carry;
        FlagsOut[1] <= FunSel[4] ? ALUOut[31] : ALUOut[15];
        FlagsOut[0] <= overflow
    end
    
endmodule
