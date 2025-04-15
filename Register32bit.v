`timescale 1ns / 1ps
module Register32bit(
        input wire [31:0] I,          
        input wire [2:0] FunSel,      
        input wire E,                
        input wire Clock,               
        output reg [31:0] Q         
    );

    always @(posedge Clock) begin
        if (E) begin
            case (FunSel)
                3'b000: Q <= Q - 1;    
                3'b001: Q <= Q + 1;    
                3'b010: Q <= I;                       
                3'b011: begin Q <= 32'b0;end               
                3'b100: begin  Q <= {24'b0, I[7:0]};end         
                3'b101: begin  Q <= {16'b0, I[15:0]};end        
                3'b110: begin  Q <= {Q[23:0], I[7:0]};end
                3'b111: begin  Q <= {{16{I[15]}}, I[15:0]};end
                default: Q <= Q;
            endcase
        end

    end
endmodule