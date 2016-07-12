#include "type_defs.h"

// Write PoseTransform to file
std::ofstream& operator<<(std::ofstream& os, const PoseTransform& obj)
{
    for (int i = 0; i < 3; ++i) {
        os.write(reinterpret_cast<const char*>(obj.T+i), sizeof(double));
        os.write(reinterpret_cast<const char*>(obj.R+i), sizeof(double));
        os.write(reinterpret_cast<const char*>(obj.V+i), sizeof(double));
    }
    os.write(reinterpret_cast<const char*>(&(obj.timestamp)), sizeof(obj.timestamp));
    os.write(reinterpret_cast<const char*>(&(obj.week_number)), sizeof(obj.week_number));
    os.write(reinterpret_cast<const char*>(&(obj.milliseconds)), sizeof(obj.milliseconds));
    os.write(reinterpret_cast<const char*>(&(obj.week_number_pos)), sizeof(obj.week_number_pos));
    os.write(reinterpret_cast<const char*>(&(obj.seconds_pos)), sizeof(obj.seconds_pos));
    return os;
}

// Read PoseTransform from file
std::ifstream& operator>>(std::ifstream& is, PoseTransform& obj)
{
    for (int i = 0; i < 3; ++i) {
        is.read(reinterpret_cast<char*>(obj.T+i), sizeof(double));
        is.read(reinterpret_cast<char*>(obj.R+i), sizeof(double));
        is.read(reinterpret_cast<char*>(obj.V+i), sizeof(double));
    }
    is.read(reinterpret_cast<char*>(&(obj.timestamp)), sizeof(obj.timestamp));
    is.read(reinterpret_cast<char*>(&(obj.week_number)), sizeof(obj.week_number));
    is.read(reinterpret_cast<char*>(&(obj.milliseconds)), sizeof(obj.milliseconds));
    is.read(reinterpret_cast<char*>(&(obj.week_number_pos)), sizeof(obj.week_number_pos));
    is.read(reinterpret_cast<char*>(&(obj.seconds_pos)), sizeof(obj.seconds_pos));
    return is;
}

// Write PoseTransform to std::cout for DEBUG
std::ostream& operator<<(std::ostream& os, const PoseTransform& obj)
{
    os << "Translation (ENU): (" << obj.T[0] << ", " << obj.T[1] << ", " << obj.T[2] << ")\n";
    os << "Rotation (RPY): (" << obj.R[0] << ", " << obj.R[1] << ", " << obj.R[2] << ")\n";
    os << "Velocity (ENU): (" << obj.V[0] << ", " << obj.V[1] << ", " << obj.V[2] << ")\n";
    os << "Processing time: " << obj.timestamp << '\n';
    os << "Time of packet send: " << obj.week_number << " weeks, " << obj.milliseconds << " millisecs\n";
    os << "Time of Pose: " << obj.week_number_pos << " weeks, " << obj.seconds_pos << " seconds\n";
    return os;
}

PoseTransform::PoseTransform()
{
    for (int i = 0; i < 3; ++i) {
        T[i] = 0;
        R[i] = 0;
        V[i] = 0;
    }
    week_number = 0;
    milliseconds = week_number_pos = 0;
    seconds_pos = -1; // default value -1 to indicate not a valid PoseTransform
}

//std::ifstream &operator>>(std::ifstream &is, TimeLine<PoseTransform> &obj)
//{
//    boost::shared_ptr<PoseTransform> item(new PoseTransform());
//    while (is >> *(item.get())) {
//        obj.addData(item);
//        item = boost::shared_ptr<PoseTransform>(new PoseTransform());
//    }
//    return is;
//}
