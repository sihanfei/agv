#include "protocol.h"
using namespace std;

ObjectData object_data;                 //传感器产生的单个object的原始数据
ObjectStr0 object_str_0;                //转换后的Object_Data_0
ObjectStr1 object_str_1;                //转换后的Object_Data_1
struct can_frame can_obj_0, can_obj_1;  // CAN报文

ObjectData object_data_parser;   //解析后的单个object数据
ObjectStr0 object_str_0_parser;  //解析后的Object_Data_0
ObjectStr1 object_str_1_parser;  //解析后的Object_Data_1

//临时赋值
void assignment(void)
{
  object_data.id = 6;
  object_data.obj_class = CAR;
  object_data.position[0] = -10.5;
  object_data.position[1] = -13.4;
  object_data.velocity = -12.3;
  object_data.confidence = 0.85;
  object_data.width = 3.1;
  for (size_t i = 0; i < 4; i++)
  {
    object_data.polygon[i] = i * 4;
  }
  printf("<<<真值\n");
  printf("ID: %d\n", object_data.id);
  printf("Class: %d\n", object_data.obj_class);
  printf("Position: [%.1f,%.1f]\n", object_data.position[0], object_data.position[1]);
  printf("Velocity: %.1f\n", object_data.velocity);
  printf("Confidence: %.2f\n", object_data.confidence);
  printf("Width: %.1f\n", object_data.width);
  for (int i = 0; i < 4; i++)
  {
    printf("Polygon_%d: %d\n", i, object_data.polygon[i]);
  }
}

//换算
void transform(void)
{
  object_str_0.id = object_data.id;
  object_str_0.obj_class = object_data.obj_class;
  object_str_0.confidence = (uint)(object_data.confidence * 100);  // 0-100
  object_str_0.position_x = (int)(object_data.position[0] * 10);
  object_str_0.position_y = (int)(object_data.position[1] * 10);
  object_str_0.velocity = (int)(object_data.velocity * 10);

  object_str_1.id = object_data.id;
  object_str_1.polygon_x_min = object_data.polygon[0];
  object_str_1.polygon_x_max = object_data.polygon[1];
  object_str_1.polygon_y_min = object_data.polygon[2];
  object_str_1.polygon_y_max = object_data.polygon[3];
  object_str_1.width = (uint)(object_data.width * 10);
}

//装填
void write(void)
{
  can_obj_0.can_id = 211;
  can_obj_0.can_dlc = 8;
  can_obj_0.data[0] = object_str_0.id & 0xFF;
  can_obj_0.data[1] = object_str_0.obj_class & 0xFF;
  can_obj_0.data[2] = object_str_0.confidence & 0xFF;
  can_obj_0.data[3] = object_str_0.velocity >> 2 & 0xFF;
  can_obj_0.data[4] = object_str_0.velocity << 6 & 0xC0;
  can_obj_0.data[5] = object_str_0.position_x >> 4 & 0xFF;
  can_obj_0.data[6] = (object_str_0.position_x << 4 & 0xF0) | (object_str_0.position_y >> 8 & 0x0F);
  can_obj_0.data[7] = object_str_0.position_y & 0xFF;

  can_obj_1.can_id = 212;
  can_obj_1.can_dlc = 8;
  can_obj_1.data[0] = object_str_1.id & 0xFF;
  can_obj_1.data[1] = object_str_1.width >> 4 & 0xFF;
  can_obj_1.data[2] = (object_str_1.width << 4 & 0xF0) | (object_str_1.polygon_y_min >> 6 & 0x0F);
  can_obj_1.data[3] = (object_str_1.polygon_y_min << 2 & 0xFC) | (object_str_1.polygon_y_max >> 8 & 0x03);
  can_obj_1.data[4] = object_str_1.polygon_y_max & 0xFF;
  can_obj_1.data[5] = object_str_1.polygon_x_min >> 4 & 0xFF;
  can_obj_1.data[6] = (object_str_1.polygon_x_min << 4 & 0xF0) | (object_str_1.polygon_x_max >> 8 & 0x0F);
  can_obj_1.data[7] = object_str_1.polygon_x_max & 0xFF;
}

//解析
void parse(void)
{
  object_str_0_parser.id = can_obj_0.data[0];
  object_str_0_parser.obj_class = can_obj_0.data[1];
  object_str_0_parser.confidence = can_obj_0.data[2];
  object_str_0_parser.position_x = (can_obj_0.data[5] << 4) | (can_obj_0.data[6] >> 4 & 0x0F);
  object_str_0_parser.position_y = ((can_obj_0.data[6] & 0x0F) << 8) | can_obj_0.data[7];
  object_str_0_parser.velocity = (can_obj_0.data[3] << 2) | (can_obj_0.data[4] >> 6 & 0x03);

  object_str_1_parser.id = can_obj_1.data[0];
  object_str_1_parser.width = (can_obj_1.data[1] << 4) | (can_obj_1.data[2] >> 4 & 0x0F);
  object_str_1_parser.polygon_x_min = (can_obj_1.data[5] << 4) | (can_obj_1.data[6] >> 4 & 0x0F);
  object_str_1_parser.polygon_x_max = (can_obj_1.data[6] & 0x0F) << 8 | (can_obj_1.data[7]);
  object_str_1_parser.polygon_y_min = (can_obj_1.data[2] & 0x0F) << 6 | (can_obj_1.data[3] >> 2 & 0xCF);
  object_str_1_parser.polygon_y_max = (can_obj_1.data[3] & 0x03) << 8 | can_obj_1.data[4];

  object_data_parser.id = object_str_0_parser.id;
  object_data_parser.obj_class = class_enum(object_str_0_parser.obj_class);
  object_data_parser.position[0] = object_str_0_parser.position_x * 0.1;
  object_data_parser.position[1] = object_str_0_parser.position_y * 0.1;
  object_data_parser.velocity = object_str_0_parser.velocity * 0.1;
  object_data_parser.confidence = (float)object_str_0_parser.confidence * 0.01;

  object_data_parser.width = object_str_1_parser.width * 0.1;
  object_data_parser.polygon[0] = object_str_1_parser.polygon_x_min;
  object_data_parser.polygon[1] = object_str_1_parser.polygon_x_max;
  object_data_parser.polygon[2] = object_str_1_parser.polygon_y_min;
  object_data_parser.polygon[3] = object_str_1_parser.polygon_y_max;

  printf("--------------------------------------------------------\n");
  printf("<<<解析值\n");
  printf("ID: %d\n", object_data_parser.id);
  printf("Class: %d\n", object_data_parser.obj_class);
  printf("Position: [%.1f,%.1f]\n", object_data_parser.position[0], object_data_parser.position[1]);
  printf("Velocity: %.1f\n", object_data_parser.velocity);
  printf("Confidence: %.2f\n", object_data_parser.confidence);
  printf("Width: %.1f\n", object_data_parser.width);
  for (int i = 0; i < 4; i++)
  {
    printf("Polygon_%d: %d\n", i, object_data_parser.polygon[i]);
  }
}

int main(int argc, char **argv)
{
  assignment();
  transform();
  write();
  parse();
  return 0;
}
