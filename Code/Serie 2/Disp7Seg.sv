    `timescale 100ns / 1ps
    
    module Disp7Seg(   
        input logic clk,
        input logic [3:0] d3, d2, d1, d0,
        output logic [3:0] enabled,
        output logic [6:0] ag
    );
    
    logic [16:0] conteo;
    logic [1:0] fsm;
    logic [3:0] display_number;
    
    
    // Flicker Rate
    always_ff @(posedge clk)
        if (conteo == 100000 - 1) begin
            fsm <= fsm + 1;
            conteo <= 0;
        end else begin
            conteo <= conteo + 1;
        end
     
    // Display Mux
    always_comb
        case(fsm)
            2'b00: begin enabled = 4'b1110; display_number = d0; end
            2'b01: begin enabled = 4'b1101; display_number = d1; end
            2'b10: begin enabled = 4'b1011; display_number = d2; end
            2'b11: begin enabled = 4'b0111; display_number = d3; end
        endcase
    
    // Display output value 
    always_comb
        case(display_number)
                4'd0: ag = 7'b1000000;
                4'd1: ag = 7'b1111001;
                4'd2: ag = 7'b0100100;
                4'd3: ag = 7'b0110000;
                4'd4: ag = 7'b0011001;
                4'd5: ag = 7'b0010010;
                4'd6: ag = 7'b0000010;
                4'd7: ag = 7'b1111000;
                4'd8: ag = 7'b0000000;
                4'd9: ag = 7'b0010000;
                default: ag = 7'b1111111;
        endcase
    
    endmodule