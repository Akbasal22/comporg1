`timescale 1ns / 1ps
module InstructionRegister(
        input wire [7:0] I,          
        input wire LH,              // Which half to load
        input wire Write,            // 1 to allow write, 0 to retain
        input wire Clock,              
        output reg [15:0] IROut
    );

    always @(posedge Clock) begin
        if (Write) begin
            case (LH)
                1'b0: IROut[7:0] <= I;          // Load the lower half 
                1'b1: IROut[15:8] <= I;         // Upper 
                default: IROut <= IROut;           // Don't care
            endcase
        end
    end
endmodule