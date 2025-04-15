module Register32bit(
    input wire [31:0] I,
    input wire [2:0] FunSel,
    input wire E, Clock,
    output reg [31:0] Q
);


always @(posedge Clock) begin


    if (E)begin
    
    case(FunSel)
        3'b000:Q<=Q-1;
        3'b001:Q<=Q+1;
        3'b010:Q<=I;
        3'b011:Q<=32'b0;
        
        3'b100 : begin
            Q<={24'b0,I[7:0]};
        end
        3'b101 : begin
            Q<={16'b0,I[15:0]};  
        end
        3'b110:begin
             Q<={Q[23:0],I[7:0]}; 
        end
        3'b111:begin
              Q <= {{16{I[15]}}, I[15:0]};   
        end
        default: Q <= Q;
    endcase
    
    end
end

endmodule

module RegisterFile(
    input wire [31:0] I,
    input wire Clock,
    input wire [2:0] OutASel, OutBSel, FunSel,
    input wire [3:0] RegSel, ScrSel,
    output reg [31:0] OutA, OutB
);
wire R1E;
wire R2E;
wire R3E;
wire R4E;

wire S1E;
wire S2E;
wire S3E;
wire S4E;

wire [31:0] R1Q;
wire [31:0] R2Q;
wire [31:0] R3Q;
wire [31:0] R4Q;

wire [31:0] S1Q;
wire [31:0] S2Q;
wire [31:0] S3Q;
wire [31:0] S4Q;

    assign {R1E,R2E,R3E,R4E} = RegSel;
    assign {S1E,S2E,S3E,S4E} = ScrSel;
    
always @(*) begin
        case (OutASel)
            3'b000: OutA = R1Q;
            3'b001: OutA = R2Q;
            3'b010: OutA = R3Q;
            3'b011: OutA = R4Q;
            3'b100: OutA = S1Q;
            3'b101: OutA = S2Q;
            3'b110: OutA = S3Q;
            3'b111: OutA = S4Q;
            default: OutA = 32'b0;
        endcase
        case (OutBSel)
            3'b000: OutB = R1Q;
            3'b001: OutB = R2Q;
            3'b010: OutB = R3Q;
            3'b011: OutB = R4Q;
            3'b100: OutB = S1Q;
            3'b101: OutB = S2Q;
            3'b110: OutB = S3Q;
            3'b111: OutB = S4Q;
            default: OutB = 32'b0;
        endcase
    end

    Register32bit R1(I,FunSel,R1E,Clock,R1Q);
    Register32bit R2(I,FunSel,R2E,Clock,R2Q);
    Register32bit R3(I,FunSel,R3E,Clock,R3Q);
    Register32bit R4(I,FunSel,R4E,Clock,R4Q);

    Register32bit S1(I,FunSel,S1E,Clock,S1Q);
    Register32bit S2(I,FunSel,S2E,Clock,S2Q);
    Register32bit S3(I,FunSel,S3E,Clock,S3Q);
    Register32bit S4(I,FunSel,S4E,Clock,S4Q);
endmodule