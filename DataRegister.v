`timescale 1ns / 1ps
module DataRegister(
        input wire [7:0] I,          
        input wire [1:0] FunSel,     
        input wire E,                 
        input wire Clock,               
        output reg [31:0] DROut         
    );

    always @(posedge Clock) begin
        if (E) begin
            case (FunSel)
                2'b00: begin                    // Sign extend and load
                    DROut[31:8] <= {24{I[7]}};     
                    DROut[7:0]  <= I;              
                end
                2'b01: begin                    // Clear upper 24 bits and load I to the rest
                    DROut[31:8] <= 24'b0;        
                    DROut[7:0]  <= I;            
                end
                2'b10: begin                    // Left shift by 8 bits and load I
                    DROut[31:8] <= {DROut[23:0], 8'b0};   
                    DROut[7:0]  <= I;                 
                end
                2'b11: begin                    // Right shift by 8 bits, then load 
                    DROut[31:8] <= {8'b0, DROut[31:8]};  
                    DROut[31:24] <= I;                    
                end
                default: begin
                    DROut <= DROut;
                end
            endcase
        end
    end
endmodule