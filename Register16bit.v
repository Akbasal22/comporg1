`timescale 1ns / 1ps
module Register16bit(
        input wire [15:0] I,          
        input wire [1:0] FunSel,     
        input wire E,                 
        input wire Clock,               
        output reg [15:0] Q           
    );

    always @(posedge Clock) begin
        if (E) begin
            case (FunSel)
                2'b00: Q <= Q - 1;    
                2'b01: Q <= Q + 1;    
                2'b10: Q <= I;        
                2'b11: Q <= 16'b0;   
            endcase
        end
        // Else retain value (do nothing)
    end
endmodule

