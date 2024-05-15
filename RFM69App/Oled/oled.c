#include <oled.h>
#include <stdlib.h>
#include <ssd1306_i2c.h>




const char *filename = "/dev/i2c-1";
ssd1306_i2c_t *oled;

ssd1306_framebuffer_t *fbp;
ssd1306_framebuffer_box_t bbox;


int initialize_oled(void){
	fprintf(stderr, "DEBUG: Using library version: %s\n", ssd1306_i2c_version());
	oled = ssd1306_i2c_open(filename, 0x3c, 128, 32, NULL);
    if (!oled) {
        return -1;
    }
    if (ssd1306_i2c_display_initialize(oled) < 0) {
        fprintf(stderr, "ERROR: Failed to initialize the display. Check if it is connected !\n");
        ssd1306_i2c_close(oled);
        return -1;
    }
    sleep(3);
    fbp = ssd1306_framebuffer_create(oled->width, oled->height, oled->err);

    ssd1306_i2c_display_clear(oled);
    for (uint8_t i = oled->height - 1; i < oled->height; ++i) {
        for (uint8_t j = 0; j < oled->width; ++j)
            ssd1306_framebuffer_put_pixel(fbp, j, i, true);
    }
    ssd1306_framebuffer_hexdump(fbp);
    ssd1306_framebuffer_bitdump(fbp);
    ssd1306_framebuffer_clear(fbp);
    ssd1306_framebuffer_box_t bbox;
    
    return 0;
}
    
    
int write_text_to_oled(char* l1, char* l2){
	ssd1306_framebuffer_clear(fbp);
	ssd1306_i2c_run_cmd(oled, SSD1306_I2C_CMD_DISP_NORMAL, 0, 0);
	ssd1306_framebuffer_draw_text(fbp, l1, 0, 5, 16, SSD1306_FONT_DEFAULT, 3, &bbox);
    ssd1306_framebuffer_draw_text(fbp, l2, 0, 5, 32, SSD1306_FONT_DEFAULT, 3, &bbox);
    //ssd1306_framebuffer_bitdump(fbp);
    ssd1306_i2c_display_update(oled, fbp);
}

void invert_oled_color(void){
	ssd1306_i2c_run_cmd(oled, SSD1306_I2C_CMD_DISP_INVERTED, 0, 0);
}

void oled_set_power(uint8_t oled_state){
	if (oled_state){
		ssd1306_i2c_run_cmd(oled, SSD1306_I2C_CMD_POWER_ON, 0, 0);
	}
	else{
		ssd1306_i2c_run_cmd(oled, SSD1306_I2C_CMD_POWER_OFF, 0, 0);
	}
}

void close_oled(void){
	ssd1306_framebuffer_destroy(fbp);
    fbp = NULL;
    ssd1306_i2c_close(oled);
    oled = NULL;
}

    
