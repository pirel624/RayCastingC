#include <iostream>
#include <math.h>
#include <vector>


const short SCREEN_WIDTH = 100;
const short SCREEN_HEIGHT = 40;
const short BASELINE = SCREEN_HEIGHT / 2;

bool isRendererStarted = false;

std::vector<std::vector<char>> SCREEN_BUFFER; // Luar -> tinggi ; Dalam -> Lebar

void minimal_maximal_correction(float &lower, float &upper)
{
    if(upper >= lower)
    {
        ;
    }
    else if(upper < lower)
    {
        float buffermin = upper;
        upper = lower;
        lower = buffermin;
    }
}

void DrawBuffer(short x, short y, char data)
{
    SCREEN_BUFFER[y - 1][x - 1] = data;
}

void Draw()
{
    for(int y = 0; y < SCREEN_HEIGHT; y++)
    {
        for(int x = 0; x < SCREEN_WIDTH; x++)
        {
            std::cout << SCREEN_BUFFER[y][x];
        }
        std::cout << std::endl;
    }
}

void Clean()
{
    system("cls");
}

struct line
{
public:
    int x1, y1, x2, y2;
    float slope;                            // General Linear Equation
    float constant;
    bool XParallel = false;
    bool YParallel = false;
};

struct ray
{
    int Xpoint, Ypoint;
    int Xpivot, Ypivot;
    float slope;
    float constant;                         // General Linear Equation
    float orientation;                      // Counterclockwise
    bool XParallel = false;
    bool YParallel = false;
};

struct segment
{
    int Xstart, Ystart;
    int Xend, Yend;
    float slope;                            // General Linear Equation
    float constant;
    bool XParallel = false;
    bool YParallel = false;
};

struct point
{
    int X, Y;
    bool XUpwardTolerance = false;
    bool XDownwardTolerance = false;        // Upward tolerance means the actual value reside in range of 0.5 more
    bool YUpwardTolerance = false;          // Downward tolerance means the actual value reside in range of 0.5 less
    bool YDownwardTolerance = false;
    bool isReal = true;
};

point ProducePoint(float X, float Y)
{
    int Xrounded = X;                                   // implicit type conversion will result to accidental rounding, which we actually need
    int Yrounded = Y;

    point titik;
    titik.isReal = true;

    if(((X - Xrounded) <= 0.5) && ((X - Xrounded) > 0)){titik.X = Xrounded; titik.XUpwardTolerance = true; titik.XDownwardTolerance = false;}
    else if(((X - Xrounded) > 0.5) && ((X - Xrounded) <= 1)){titik.X = Xrounded + 1; titik.XDownwardTolerance = true; titik.XUpwardTolerance = false;}
    else if(((X - Xrounded) <= 0) && ((X - Xrounded) > -0.5)){titik.X = Xrounded; titik.XDownwardTolerance = true; titik.XUpwardTolerance = false;}
    else if(((X - Xrounded) <= -0.5) && ((X - Xrounded) > -1)){titik.X = Xrounded - 1; titik.XDownwardTolerance = false; titik.XUpwardTolerance = true;}
    else{} // undefined exception handler

    if(((Y - Yrounded) <= 0.5) && ((Y - Yrounded) > 0)){titik.Y = Yrounded; titik.YUpwardTolerance = true; titik.YDownwardTolerance = false; std::cout << "land in section 1" << "   " << titik.Y << std::endl;}
    else if(((Y - Yrounded) > 0.5) && ((Y - Yrounded) <= 1)){titik.Y = Yrounded + 1; titik.YDownwardTolerance = true; titik.YUpwardTolerance = false;  std::cout << "land in section 2" << std::endl;}
    else if(((Y - Yrounded) <= 0) && ((Y - Yrounded) > -0.5)){titik.Y = Yrounded; titik.YDownwardTolerance = true; titik.YUpwardTolerance = false;  std::cout << "land in section 3" << std::endl;}
    else if(((Y - Yrounded) <= -0.5) && ((Y - Yrounded) > -1)){titik.Y = Yrounded - 1; titik.YDownwardTolerance = false; titik.YUpwardTolerance = true;  std::cout << "land in section 4" << std::endl;}
    else{;} // undefined exception handler
    return titik;
}

point Line_Collision(line garis1, line garis2)
{
    point titik;                     // Final Product
    float Yactual;                   // The actual collision location, without implicit rounding from typecast
    float Xactual;

    if(garis1.y1 == garis1.y2){garis1.XParallel = true; garis1.YParallel = false;}    // Special Condition Flag
    if(garis1.x1 == garis1.x2){garis1.YParallel = true; garis1.XParallel = false;}
    if(garis2.x1 == garis2.x2){garis2.YParallel = true; garis2.XParallel = false;}
    if(garis2.y1 == garis2.y2){garis2.XParallel = true; garis2.YParallel = false;}

    if((!garis1.XParallel) && (!garis1.YParallel) && (!garis2.XParallel) && (!garis2.YParallel))                               // normal
    {
        garis1.slope = (float(garis1.y2) - float(garis1.y1)) / (float(garis1.x2) - float(garis1.x1));              // y = mx + c  -> c = y - mx
        garis2.slope = (float(garis2.y2) - float(garis2.y1)) / (float(garis2.x2) - float(garis2.x1));              // non axis parallel            // error note -> it produce 0  // programmer note -> division works differently for int and float
        garis1.constant = (garis1.y1) - ((garis1.slope) * (garis1.x1));
        garis2.constant = (garis2.y1) - ((garis2.slope) * (garis2.x1));
        if(garis1.slope != garis2.slope)
        {
            titik.isReal = true;
            Xactual = (float(garis2.constant) - float(garis1.constant)) / (float(garis1.slope) - float(garis2.slope));
            Yactual = Xactual * float(garis1.slope) + float(garis1.constant);
        }
        else if(garis1.slope == garis2.slope)
        {
            titik.isReal = false;           // No collision -> imaginary collision point
        }

    }
    else if(garis1.XParallel && !(garis2.XParallel || garis2.YParallel))            // c - y = -mx -> x = (y - c) / m
    {
        garis2.slope = (float(garis2.y2) - float(garis2.y1)) / (float(garis2.x2) - float(garis2.x1));
        garis2.constant = float(garis2.y1) - float(garis2.slope) * float(garis2.x1);
        Yactual = garis1.y2;
        Xactual = (float(Yactual) - float(garis2.constant)) / float(garis2.slope);
        titik.isReal = true;
    }
    else if(garis1.YParallel && !(garis2.XParallel || garis2.YParallel))            // c - y = -mx -> x = (y - c) / m -> -y = -mx - c -> y = mx + c
    {
        garis2.slope = (float(garis2.y2) - float(garis2.y1)) / (float(garis2.x2) - float(garis2.x1));
        garis2.constant = float(garis2.y1) - float(garis2.slope) * float(garis2.x1);
        Xactual = float(garis1.x1);
        Yactual = float(garis2.slope) * float(Xactual) + float(garis2.constant);
        titik.isReal = true;
    }
    else if(garis2.XParallel && !(garis1.XParallel || garis1.YParallel))
    {
        garis1.slope = (float(garis1.y2) - float(garis1.y1)) / (float(garis1.x2) - float(garis1.x1));
        garis1.constant = float(garis1.y1) - float(garis1.slope) * float(garis1.x1);
        Yactual = float(garis2.y2);
        Xactual = (float(Yactual) - float(garis1.constant)) / float(garis1.slope);
        titik.isReal = true;
    }
    else if(garis2.YParallel && !(garis1.XParallel || garis1.YParallel))
    {
        garis1.slope = (float(garis1.y2) - float(garis1.y1)) / (float(garis1.x2) - float(garis1.x1));
        garis1.constant = float(garis1.y1) - float(garis1.slope) * float(garis1.x1);
        Xactual = float(garis2.x1);
        Yactual = float(garis1.slope) * float(Xactual) + float(garis1.constant);
        titik.isReal = true;
    }
    else if(garis1.XParallel && garis2.XParallel)
    {
        titik.isReal = false;
    }
    else if(garis1.XParallel && garis2.YParallel)
    {
        Xactual = garis2.x1;
        Yactual = garis1.y1;
    }
    else if(garis1.YParallel && garis2.XParallel)
    {
        Xactual = garis1.x1;
        Yactual = garis2.y1;
    }
    else if(garis1.YParallel && garis2.YParallel)
    {
        titik.isReal = false;
    }

    if(titik.isReal)
    {
        int Xrounded = Xactual;                                   // implicit type conversion will result to accidental rounding, which we actually need
        int Yrounded = Yactual;

        if(((Xactual - Xrounded) <= 0.5) && ((Xactual - Xrounded) > 0)){titik.X = Xrounded; titik.XUpwardTolerance = true; titik.XDownwardTolerance = false;}
        else if(((Xactual - Xrounded) > 0.5) && ((Xactual - Xrounded) <= 1)){titik.X = Xrounded + 1; titik.XDownwardTolerance = true; titik.XUpwardTolerance = false;}
        else if(((Xactual - Xrounded) <= 0) && ((Xactual - Xrounded) > -0.5)){titik.X = Xrounded; titik.XDownwardTolerance = true; titik.XUpwardTolerance = false;}
        else if(((Xactual - Xrounded) <= -0.5) && ((Xactual - Xrounded) > -1)){titik.X = Xrounded - 1; titik.XDownwardTolerance = false; titik.XUpwardTolerance = true;}
        else{} // undefined exception handler

        if(((Yactual - Yrounded) <= 0.5) && ((Yactual - Yrounded) > 0)){titik.Y = Yrounded; titik.YUpwardTolerance = true; titik.YDownwardTolerance = false; std::cout << "land in section 1" << "   " << titik.Y << std::endl;}
        else if(((Yactual - Yrounded) > 0.5) && ((Yactual - Yrounded) <= 1)){titik.Y = Yrounded + 1; titik.YDownwardTolerance = true; titik.YUpwardTolerance = false;  std::cout << "land in section 2" << std::endl;}
        else if(((Yactual - Yrounded) <= 0) && ((Yactual - Yrounded) > -0.5)){titik.Y = Yrounded; titik.YDownwardTolerance = true; titik.YUpwardTolerance = false;  std::cout << "land in section 3" << std::endl;}
        else if(((Yactual - Yrounded) <= -0.5) && ((Yactual - Yrounded) > -1)){titik.Y = Yrounded - 1; titik.YDownwardTolerance = false; titik.YUpwardTolerance = true;  std::cout << "land in section 4" << std::endl;}
        else{;} // undefined exception handler
        return titik;
    }
    else if(!titik.isReal)
    {
        return titik;
    }
}

line ConstructLineFromRay(ray sinar)
{
    line garis;
    garis.x1 = sinar.Xpoint;
    garis.y1 = sinar.Ypoint;
    garis.x2 = sinar.Xpivot;
    garis.y2 = sinar.Ypivot;
    return garis;
}

line ConstructLineFromSegment(segment ruas)
{
    line garis;
    garis.x1 = ruas.Xstart;
    garis.y1 = ruas.Ystart;
    garis.x2 = ruas.Xend;
    garis.y2 = ruas.Yend;
    return garis;
}

bool isPointInRay(point titik, ray sinar)   // Dev note = {Int division need to be avoided}
{
    bool isInline = false;
    if(sinar.Ypoint == sinar.Ypivot){sinar.XParallel = true; sinar.YParallel = false;}           // Special Condition Flag
    else if(sinar.Xpoint == sinar.Xpivot){sinar.YParallel = true; sinar.XParallel = false;}
    else{sinar.XParallel == false; sinar.XParallel = false;}
    std::cout << "Apakah normal : " << !(sinar.XParallel || sinar.YParallel) << std::endl;
    std::cout << "Apakah paralel x : " << sinar.XParallel << std::endl;
    std::cout << "Apakah paralel y : " << sinar.YParallel << std::endl;

    if(!(sinar.XParallel || sinar.YParallel))                // normal
    {
        std::cout << "Ray's slope is normal" << std::endl;
        sinar.slope = (sinar.Ypivot - sinar.Ypoint) / (sinar.Xpivot - sinar.Ypivot);
        sinar.constant = sinar.Ypoint - sinar.slope * sinar.Xpoint;
        float Ymin;
        float Ymax;
        if(titik.XDownwardTolerance)
        {
            Ymin = sinar.slope * (titik.X - 0.5) + sinar.constant;                    // Because we only know the range of the value of x, we cannot exactly determine the value of y
            Ymax = sinar.slope * (titik.X) + sinar.constant;
        }
        else if(titik.XUpwardTolerance)
        {
            Ymin = sinar.slope * (titik.X) + sinar.constant;
            Ymax = sinar.slope * (titik.X + 0.5) + sinar.constant;
        }
        else{;} // undefined exception

        float Yactualmin;
        float Yactualmax;
        if(titik.YDownwardTolerance)
        {
            Yactualmax = titik.Y;
            Yactualmin = titik.Y - 0.5;
        }
        else if(titik.YUpwardTolerance)
        {
            Yactualmax = titik.Y + 0.5;
            Yactualmin = titik.Y;
        }
        else{;} // undefined exception

        minimal_maximal_correction(Ymin, Ymax);
        minimal_maximal_correction(Yactualmin, Yactualmax);
        float length_sum = (Ymax - Ymin) + (Yactualmax - Yactualmin);
        minimal_maximal_correction(Ymin, Yactualmin);
        minimal_maximal_correction(Ymax, Yactualmax);
        float length_actual = Ymax - Ymin;
        if(length_actual <= length_sum)
        {
            isInline = true;
        }
        else if(length_actual > length_sum)
        {
            isInline = false;
        }
    }
    else if(sinar.XParallel && !sinar.YParallel)          // Stationary on y axis
    {
        std::cout << "Ray parallel to x axis" << std::endl;
        float Ystation = sinar.Ypoint;
        float Ymin;
        float Ymax;
        if(titik.YDownwardTolerance)
        {
            Ymin = titik.Y - 0.5;
            Ymax = titik.Y;
        }
        else if(titik.YUpwardTolerance)
        {
            Ymin = titik.Y;
            Ymax = titik.Y + 0.5;
        }
        else{;} // Undefined exception handler

        minimal_maximal_correction(Ymin, Ymax);

        if(Ymin <= Ystation <= Ymax)
        {
            isInline = true;
        }
        else
        {
            isInline = false;
        }
    }
    else if(sinar.YParallel && !sinar.XParallel)          // Stationary on x axis
    {
        std::cout << "Ray parallel to y axis" << std::endl;
        float Xstation = sinar.Xpoint;
        float Xmin;
        float Xmax;
        if(titik.YDownwardTolerance)
        {
            Xmin = titik.X - 0.5;
            Xmax = titik.X;
        }
        else if(titik.YUpwardTolerance)
        {
            Xmin = titik.X;
            Xmax = titik.X + 0.5;
        }
        else{;} // Undefined exception handler

        minimal_maximal_correction(Xmin, Xmax);

        if(Xmin <= Xstation <= Xmax)
        {
            isInline = true;
        }
        else
        {
            isInline = false;
        }
    }

    if(isInline)
    {
        float Ymin = sinar.Ypoint;
        float Ymax = sinar.Ypivot;
        float Xmin = sinar.Xpoint;
        float Xmax = sinar.Xpivot;

        float Xthreshold1 = titik.X;
        float Xthreshold2;
        if(titik.XDownwardTolerance){Xthreshold2 = titik.X - 0.5;}
        if(titik.XUpwardTolerance){Xthreshold2 = titik.X + 0.5;}

        float Ythreshold1 = titik.Y;
        float Ythreshold2;
        if(titik.YDownwardTolerance){Ythreshold2 = titik.Y - 0.5;}
        if(titik.YUpwardTolerance){Ythreshold2 = titik.Y + 0.5;}


        if(((((Ymax - Ymin) * (Ythreshold1 - Ymin)) >= 0) || (((Ymax - Ymin) * (Ythreshold2 - Ymin)) >= 0)) && ((((Xmax - Xmin) * (Xthreshold1 - Xmin)) >= 0) || (((Xmax - Xmin) * (Xthreshold2 - Xmin)) >= 0)))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else if(!isInline)
    {
        return false;
    }

}

bool isPointInSegment(point titik, segment ruas)
{
    ray sinar1;
    ray sinar2;

    sinar1.Xpoint = ruas.Xstart;    sinar2.Xpoint = ruas.Xstart;
    sinar1.Ypoint = ruas.Ystart;    sinar2.Ypoint = ruas.Ystart;
    sinar1.Xpivot = ruas.Xend;      sinar2.Xpivot = ruas.Xend;
    sinar1.Ypivot = ruas.Yend;      sinar2.Ypivot = ruas.Yend;

    if(isPointInRay(titik, sinar1) && isPointInRay(titik, sinar2))
    {
        return true;
    }
    else
    {
        return false;
    }
}

point RaySegmentCollision(ray sinar, segment ruas)
{
    line garis1 = ConstructLineFromSegment(ruas);
    line garis2 = ConstructLineFromRay(sinar);
    point tabrakan = Line_Collision(garis1, garis2);
    if(tabrakan.isReal)
    {
        if(isPointInRay(tabrakan, sinar) && isPointInSegment(tabrakan, ruas))
        {
            return tabrakan;
        }
        else
        {
            tabrakan.isReal = false;
            return tabrakan;
        }
    }
    else
    {
        tabrakan.isReal = false;
        return tabrakan;
    }
}

int main()
{
    /*if(!isRendererStarted)
    {
        std::vector<char> temporary_data_bucket;
        for(int n = 0; n < SCREEN_HEIGHT; n++)
        {
            temporary_data_bucket.clear();
            for(int y = 0; y < SCREEN_WIDTH; y++)
                {
                    temporary_data_bucket.push_back(' ');                                                // "Bleach" the screen with space character
                }
            SCREEN_BUFFER.push_back(temporary_data_bucket);
        }
        isRendererStarted = true;
    }*/

   /* line garis1;
    line garis2;
    point tabrakan;
    while(true)
    {
        std::cout << "koordinat L1X1 : ";
        std::cin >> garis1.x1;
        std::cout << "koordinat L1X2 : ";
        std::cin >> garis1.x2;
        std::cout << "koordinat L1Y1 : ";
        std::cin >> garis1.y1;
        std::cout << "koordinat L1Y2 : ";
        std::cin >> garis1.y2;

        std::cout << "koordinat L2X1 : ";
        std::cin >> garis2.x1;
        std::cout << "koordinat L2X2 : ";
        std::cin >> garis2.x2;
        std::cout << "koordinat L2Y1 : ";
        std::cin >> garis2.y1;
        std::cout << "koordinat L2Y2 : ";
        std::cin >> garis2.y2;

        tabrakan = Line_Collision(garis1, garis2);

        std::cout << "Apakah ada tabrakan : " << tabrakan.isReal << std::endl;
        std::cout << "Basis nilai X : " << tabrakan.X << std::endl;\
        std::cout << "X Upward Tolerance : " << tabrakan.XUpwardTolerance << std::endl;
        std::cout << "X Downward Tolerance : " << tabrakan.XDownwardTolerance << std::endl;
        std::cout << "Basis nilai Y : " << tabrakan.Y << std::endl;
        std::cout << "Y Upward Tolerance : " << tabrakan.YUpwardTolerance << std::endl;
        std::cout << "Y Downward Tolerance : " << tabrakan.YDownwardTolerance << std::endl;



    }*/

    point titik1, titik2, titik3, titik4;
    titik1 = ProducePoint(113, 9);
    titik2 = ProducePoint(111, 22);
    titik3 = ProducePoint(98, 43);
    titik4 = ProducePoint(78, 44);

    std::cout << "Apakah ada tabrakan : " << titik1.isReal << std::endl;
    std::cout << "Basis nilai X : " << titik1.X << std::endl;\
    std::cout << "X Upward Tolerance : " << titik1.XUpwardTolerance << std::endl;
    std::cout << "X Downward Tolerance : " << titik1.XDownwardTolerance << std::endl;
    std::cout << "Basis nilai Y : " << titik1.Y << std::endl;
    std::cout << "Y Upward Tolerance : " << titik1.YUpwardTolerance << std::endl;
    std::cout << "Y Downward Tolerance : " << titik1.YDownwardTolerance << std::endl;


    ray sinar;
    sinar.Xpoint = 90;
    sinar.Ypoint = 32;
    sinar.Xpivot = 109;
    sinar.Ypivot = 13;

    std::cout << "Titik 1 : " << isPointInRay(titik1, sinar) << std::endl;
    std::cout << "Titik 2 : " << isPointInRay(titik2, sinar) << std::endl;
    std::cout << "Titik 3 : " << isPointInRay(titik3, sinar) << std::endl;
    std::cout << "Titik 4 : " << isPointInRay(titik4, sinar) << std::endl;
}
