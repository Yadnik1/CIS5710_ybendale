/* INSERT NAME AND PENNKEY HERE */

`timescale 1ns / 1ns

// quotient = dividend / divisor

module divu_1iter (
    input  wire [31:0] i_dividend,
    input  wire [31:0] i_divisor,
    input  wire [31:0] i_remainder,
    input  wire [31:0] i_quotient,
    output wire [31:0] o_dividend,
    output wire [31:0] o_remainder,
    output wire [31:0] o_quotient
);
    wire [31:0] temp_remainder;
    wire [31:0] temp_remainder1;
    wire [31:0] temp_remainder3;
    wire [31:0] temp_remainder2;
    wire x;

    assign temp_remainder = {i_remainder[30:0], 1'b0};
    assign temp_remainder1 = {31'b0, i_dividend[31]};
    assign temp_remainder2 = temp_remainder1 & {31'b0, 1'b1};
    assign temp_remainder3 = temp_remainder | temp_remainder2;

    assign x = (temp_remainder3 < i_divisor);

    assign o_quotient = (x) ? {i_quotient[30:0], 1'b0} : {i_quotient[30:0], 1'b1}; 
    assign o_remainder = (x) ? temp_remainder3 : (temp_remainder3 - i_divisor);
    assign o_dividend = {i_dividend[30:0], 1'b0};

endmodule

module divider_unsigned_pipelined (
    input wire clk, rst,
    input  wire [31:0] i_dividend,
    input  wire [31:0] i_divisor,
    output wire [31:0] o_remainder,
    output wire [31:0] o_quotient

);

    logic [31:0] remainder [33];
    logic [31:0] quotient [33] ;
    logic [31:0] dividend [33]; 
    logic [31:0] iremainder ;
    logic [31:0] iquotient  ;
    logic [31:0] idividend ;     
    logic [31:0] idivisor;
    assign dividend [0] = i_dividend;
    assign quotient [0] = 32'b0;
    assign remainder [0] = 32'b0;
    genvar i;
    for (i = 0; i < 32; i = i + 1) begin : vedant
        if (i==16) begin : manas
            divu_1iter d1(
            .i_dividend(idividend),
            .i_divisor(idivisor),
            .i_remainder(iremainder),
            .i_quotient(iquotient),
            .o_dividend(dividend[i+1]),
            .o_remainder(remainder[i+1]),
            .o_quotient(quotient[i+1])
            );
        end else begin : chips
            divu_1iter d2(
                .i_dividend(dividend[i]),
                .i_divisor((i<16)?i_divisor: idivisor),
                .i_remainder(remainder[i]),
                .i_quotient(quotient[i]),
                .o_dividend(dividend[i+1]),
                .o_remainder(remainder[i+1]),
                .o_quotient(quotient[i+1])
            );
        end
        if (i==15) begin : design_chips
            always_ff @ (posedge clk) begin
                if(rst) begin : bnm
                idividend <= 32'b0;
                iremainder <= 32'b0;
                iquotient <= 32'b0;
                idivisor <= 32'b0;
                end else begin
                idividend <= dividend[i+1];
                iremainder <= remainder[i+1];
                iquotient <= quotient[i+1];
                idivisor <= i_divisor;
                end
            end
        end
    end

    assign o_remainder = remainder[32];
    assign o_quotient = quotient[32];

endmodule
