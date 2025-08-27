#ifndef __COLOR_PIECE_H
#define __COLOR_PIECE_H

typedef struct {
	char color;     //颜色
	int  centre_x;  //中心X值
	int  centre_y;  //中心Y值
	int  rect_x;    //左上X值
	int  rect_y;    //左上Y值
	int  rect_w;    //宽度
	int  rect_h;    //高度
} Color_Info;

void OpenColor(void);


































#endif


