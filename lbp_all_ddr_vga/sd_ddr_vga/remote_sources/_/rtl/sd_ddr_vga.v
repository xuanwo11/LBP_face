`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name:    sd_ddr_vga 
//////////////////////////////////////////////////////////////////////////////////
module sd_ddr_vga(
   input clk_50M,
	input reset_n,
   output [3:0] led,                  //led灯指示

   //DDR的接口信号
   inout  [15:0]            mcb3_dram_dq,
   output [13:0]            mcb3_dram_a,
   output [2:0]             mcb3_dram_ba,
   output                   mcb3_dram_ras_n,
   output                   mcb3_dram_cas_n,
   output                   mcb3_dram_we_n,
   output                   mcb3_dram_odt,
   output                   mcb3_dram_cke,
   output                   mcb3_dram_reset_n,
   output                   mcb3_dram_dm,
   inout                    mcb3_dram_udqs,
   inout                    mcb3_dram_udqs_n,
   inout                    mcb3_rzq,
   inout                    mcb3_zio,
   output                   mcb3_dram_udm,
   inout                    mcb3_dram_dqs,
   inout                    mcb3_dram_dqs_n,
   output                   mcb3_dram_ck,
   output                   mcb3_dram_ck_n,
	
	 //VGA的接口信号
	output [4:0]            vga_r,
   output [5:0]            vga_g,
   output [4:0]            vga_b,
   output                  vga_hsync,
   output                  vga_vsync,
	
	//SD卡SPI接口
	output                  SD_clk,
	output                  SD_cs,
	output                  SD_datain,
	input                   SD_dataout
		
    );

wire [7:0] mydata_o;                             //sd data
wire myvalid_o;                                  //sd data valid
wire init_o;                                     //sd卡初始化完成信号
wire pic_read_done;                              //sd卡读图像完成信号         

wire [255:0] ddr_data;

wire ddr_rd_cmd;
wire ddr_rden;

wire lbp_finish;
wire lbp_wrend_flag;
assign led[0]=init_o?1'b0:1'b1;                //led0为ddr calibrate完成指示信号,亮说明初始化完成
assign led[1]=pic_read_done?1'b0:1'b1;             //led1为图像已存入DDR中完成指示信号,亮说明存储已完成
assign led[2]=lbp_finish?1'b0:1'b1;//c3_p0_wr_full?1'b0:1'b1;             //led2为写数据不足指示信号,亮说明错误
assign led[3]=lbp_wrend_flag?1'b0:1'b1;//c3_p0_cmd_full?1'b0:1'b1;            //led3为读数据over指示信号,亮说明错误

	
wire [5:0]  ddr_read_state;                         //for chipscope debug
wire [29:0] c3_p0_cmd_byte_addr;                    //for chipscope debug
wire			c3_p0_cmd_en;                           //for chipscope debug
wire [2:0]	c3_p0_cmd_instr;                        //for chipscope debug
wire [5:0]	c3_p0_cmd_bl;                           //for chipscope debug
wire        c3_p0_rd_en;                            //for chipscope debug 
wire        c3_clk0;
wire [127:0] c3_p0_rd_data;


wire [4:0] ddr_r_reg;
wire [5:0] ddr_g_reg;
wire [4:0] ddr_b_reg;

            
//SD card ctrl system

sd_top u_sd_top
(
	//Global Clock
	.sd_rstn  		   (~c3_rst0),	     //global reset
	
	//SD SPI Interface
	.SD_clk			   (SD_clk),         //SD SPI clock 
	.SD_cs			   (SD_cs),          //SD SPI CS	
	.SD_datain			(SD_datain),		//SD SPI data in	
	.SD_dataout			(SD_dataout),		//SD SPI data out

	//Ouput SD Data                 
	.myvalid_o		   (myvalid_o),		//sd data valid
	.mydata_o			(mydata_o),  	   //sd data
	.init_o			   (init_o),	
	.pic_read_done    (pic_read_done)
	
);


//VGA显示控制部分
vga_disp	vga_disp_inst(
	.vga_clk             (vga_clk),
	.ddr_data            (c3_p0_rd_data),
	.vga_r               (vga_r),
	.vga_g               (vga_g),
	.vga_b               (vga_b),
	.vga_hsync           (vga_hsync),	
	.vga_vsync           (vga_vsync),
	.ddr_addr_set        (ddr_addr_set),
	.ddr_rden            (ddr_rden),
	.ddr_r_reg           (ddr_r_reg),
	.ddr_g_reg           (ddr_g_reg),
	.ddr_b_reg           (ddr_b_reg),
	.hsync_de            (hsync_de),
	.ddr_rd_cmd          (ddr_rd_cmd)
	
);

//DDR读写控制部分
ddr_rw ddr_rw_inst(
	.clk_50M             (clk_50M),	
	.spi_clk             (SD_clk),
   .vga_clk             (vga_clk),
	.reset_n             (reset_n),
	.mydata              (mydata_o),
	.myvalid             (myvalid_o),	
	.pic_read_done       (pic_read_done),	

	.ddr_addr_set        (ddr_addr_set),
	.ddr_rden            (ddr_rden),
	.ddr_rd_cmd          (ddr_rd_cmd),
	.ddr_data            (ddr_data),

	.lbp_finish          (lbp_finish),//LBP
	.lbp_wrend_flag      (lbp_wrend_flag),//LBP
	
	.c3_p0_wr_underrun   (c3_p0_wr_underrun),	
	.c3_p0_rd_overflow   (c3_p0_rd_overflow),
	.c3_p0_wr_full       (c3_p0_wr_full),
	.c3_p0_cmd_full      (c3_p0_cmd_full),	
	.mcb3_dram_dq        (mcb3_dram_dq),	
	.mcb3_dram_a         (mcb3_dram_a),
	.mcb3_dram_ba        (mcb3_dram_ba),
	.mcb3_dram_ras_n     (mcb3_dram_ras_n),
	.mcb3_dram_cas_n     (mcb3_dram_cas_n),
	.mcb3_dram_we_n      (mcb3_dram_we_n),
	.mcb3_dram_odt       (mcb3_dram_odt),
   .mcb3_dram_reset_n   (mcb3_dram_reset_n), 	
	.mcb3_dram_cke       (mcb3_dram_cke),
	.mcb3_dram_dm        (mcb3_dram_dm),
	.mcb3_dram_udqs      (mcb3_dram_udqs),
	.mcb3_dram_udqs_n    (mcb3_dram_udqs_n),
	.mcb3_rzq            (mcb3_rzq),
	.mcb3_zio            (mcb3_zio),
	.mcb3_dram_udm       (mcb3_dram_udm),
	.c3_sys_clk          (clk_50M),
	.c3_rst0             (c3_rst0),
	.mcb3_dram_dqs       (mcb3_dram_dqs),
	.mcb3_dram_dqs_n     (mcb3_dram_dqs_n),
	.mcb3_dram_ck        (mcb3_dram_ck),	
	.mcb3_dram_ck_n      (mcb3_dram_ck_n),
	.ddr_read_state      (ddr_read_state),
	.c3_p0_cmd_byte_addr (c3_p0_cmd_byte_addr),
	.c3_p0_cmd_en        (c3_p0_cmd_en),
	.c3_p0_cmd_instr     (c3_p0_cmd_instr),	
	.c3_p0_cmd_bl        (c3_p0_cmd_bl),	
	.c3_p0_rd_en         (c3_p0_rd_en),	
	.c3_clk0             (c3_clk0),
	.c3_p0_rd_data       (c3_p0_rd_data)	

);



endmodule
