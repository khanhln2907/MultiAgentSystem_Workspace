#ifndef TIP_TYPE_H
#define TIP_TYPE_H

#define PI (double) 3.14159265359

struct Point
{
    double x;
    double y;

    // default + parameterized constructor
    Point(double x=0, double y=0) 
        : x(x), y(y)
    {
    }

    Point& operator=(const Point& a)
    {
        x = a.x;
        y = a.y;
        return *this;
    }

    Point operator-(const Point& a) const
    {
        return Point(x - a.x, y - a.y);
    }

    double dot(const Point& a) 
    {
        return x * a.x + y * a.y;
    }
};


typedef struct Line
{
    long x1;
    long y1;
    long x2;
    long y2;
}Line;

struct Vector2
{
    double x;
    double y;

    // default + parameterized constructor
    Vector2(double x=0, double y=0) 
        : x(x), y(y)
    {
    }

    Vector2 getGradient2(double U, Point v){
    	return Vector2(U/v.x, U/v.y);
    }

    Vector2 operator+(const Vector2& v)
    {
        return Vector2(x + v.x, y + v.y);
    }
};

typedef struct Pose2D {
    float x;
    float y;
}Pose2D;


typedef struct UnicycleState {
    float x;
    float y;
    float theta;
}UnicycleState;

typedef struct UnicycleInput {
    float TranVel;
    float AngularVel;
}UnicycleInput;


#endif