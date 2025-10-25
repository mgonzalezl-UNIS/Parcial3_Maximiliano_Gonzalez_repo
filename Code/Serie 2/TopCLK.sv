`timescale 100ns / 1ps

module TopCLK(
    input logic clk, reset,          // desde Basys 3
    output logic [3:0] an,            // display EN
    output logic [6:0] seg,            // segmentos a-g + dp
    output  logic dp
);

    // ------- Señales internas -------
    logic [3:0] hora_d, hora_u, min_d, min_u;

    // ------- Instancia del reloj digital -------
    Clock reloj_inst(
        .clk(clk),
        .reset(reset),
        .hora_d(hora_d),
        .hora_u(hora_u),
        .min_d(min_d),
        .min_u(min_u)
    );

    // ------- Instancia del display 7 segmentos -------
    Disp7Seg disp_inst(
        .clk(clk),          // mismo clock
        .d3(hora_d),        // display izquierdo → decimos de hora
        .d2(hora_u),        // display siguiente → unidades de hora
        .d1(min_d),         // display siguiente → decimos de minuto
        .d0(min_u),         // display derecho → unidades de minuto
        .enabled(an),       // control de displays
        .ag(seg[6:0])       // segmentos a-g
    );

    //assign dp = 1'b1; // 1 = apagado (ánodo común)

endmodule