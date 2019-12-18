#ifndef VMSCMD_H
#define VMSCMD_H


namespace pnc
{

class VMS_Cmd
{
public:

    VMS_Cmd() = default;
    ~VMS_Cmd() = default;

		VMS_Cmd(std::string id, uint8_t type,double x,double y,double heading);	// 生成VMS指令


		void setID(std::string id);
		void setType(uint8_t type);
    void setX(double x);
    void setY(double y);
    void setHeading(double heading);

		std::string getID();
		uint8_t getType();
    double getX();
    double getY();
    double getHeading();
		


private:

    std::string id_;
    uint8_t type_;
    double x_;
    double y_;
    double heading_;

};

}

#endif 
