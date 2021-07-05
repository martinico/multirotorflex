
#ifndef GPS_H
  #define GPS_H

  struct datosGPSNEO {
    float latitud; //la parte real son los grados y la decimal los minutos y segundos
    float longitud; //la parte real son los grados y la decimal los minutos y segundos
    long altitud; //metros
    unsigned long fecha; //DDMMYY format (u32)
    unsigned long hora; //HHMMSSCC format (u32)
    float velocidad; //kmh
    float direccion; //grados
    int satelites; //numero de satelites a la vista
    long HDOP; //HDOP x 100
    double VDOP; //¿VDOP?
    int declinacion; // ¡¡¡ 1,36W !!!
  };
  extern datosGPSNEO datosGPS;

  static const double declinacion = -1.36; //SACTA
  static const double MICASA_LAT = 37.401214, MICASA_LON = -5.980698; //EL DESTINO DEL VUELO :-)))

  void startGPSNEO ();
  bool testGPSNEO (unsigned long ms);
  
#endif

//https://github.com/mikalhart/TinyGPS
//http://arduiniana.org/libraries/tinygpsplus/

/*
$GPGGA - Global Positioning System Fix Data
-------------------------------------------
Name                                        Example Data        Description
Sentence Identifier                         $GPGGA              Global Positioning System Fix Data
Time                                        170834               17:08:34 Z
Latitude                                    4124.8963, N        41d 24.8963' N or 41d 24' 54" N
Longitude                                   08151.6838, W       81d 51.6838' W or 81d 51' 41" W
Fix Quality:                              - 0 = Invalid
                                          - 1 = GPS fix 
                                          - 2 = DGPS fix 
Number of Satellites                        05                  5 Satellites are in view
Horizontal Dilution of Precision (HDOP)     1.5                 Relative accuracy of horizontal position
Altitude                                    280.2, M            280.2 meters above mean sea level
Height of geoid above WGS84 ellipsoid       -34.0, M            -34.0 meters
Time since last DGPS update                 blank               No last update
DGPS reference station id                   blank               No station id
Checksum                                    *75                 Used by program to check for transmission errors

example: $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx
hhmmss.ss = UTC of position
llll.ll = latitude of position
a = N or S
yyyyy.yy = Longitude of position
a = E or W
x = GPS Quality indicator (0=no fix, 1=GPS fix, 2=Dif. GPS fix)
xx = number of satellites in use
x.x = horizontal dilution of precision
x.x = Antenna altitude above mean-sea-level
M = units of antenna altitude, meters
x.x = Geoidal separation
M = units of geoidal separation, meters
x.x = Age of Differential GPS data (seconds)
xxxx = Differential reference station ID

example: $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
1    = UTC of Position
2    = Latitude
3    = N or S
4    = Longitude
5    = E or W
6    = GPS quality indicator (0=invalid; 1=GPS fix; 2=Diff. GPS fix)
7    = Number of satellites in use [not those in view]
8    = Horizontal dilution of position
9    = Antenna altitude above/below mean sea level (geoid)
10   = Meters  (Antenna height unit)
11   = Geoidal separation (Diff between WGS-84 earth ellipsoid and mean sea level (-=geoid is below WGS-84 ellipsoid).
12   = Meters  (Units of geoidal separation)
13   = Age in seconds since last update from diff. reference station
14   = Diff. reference station ID#
15   = Checksum
*/

/*
$GPGLL - Geographic Position, Latitude / Longitude and time
-----------------------------------------------------------
eg1. $GPGLL,3751.65,S,14507.36,E*77

eg2. $GPGLL,4916.45,N,12311.12,W,225444,A
4916.46,N    Latitude 49 deg. 16.45 min. North
12311.12,W   Longitude 123 deg. 11.12 min. West
225444       Fix taken at 22:54:44 UTC
A            Data valid

eg3. $GPGLL,5133.81,N,00042.25,W*75
               1    2     3    4 5
1    5133.81   Current latitude
2    N         North/South
3    00042.25  Current longitude
4    W         East/West
5    *75       checksum

eg4. $--GLL,lll.ll,a,yyyyy.yy,a,hhmmss.ss,A llll.ll = Latitude of position
a = N or S
yyyyy.yy = Longitude of position
a = E or W
hhmmss.ss = UTC of position
A = status: A = valid data 
*/

/*
$GPGSA - GPS DOP and active satellites
--------------------------------------
eg1. $GPGSA,A,3,,,,,,16,18,,22,24,,,3.6,2.1,2.2*3C
eg2. $GPGSA,A,3,19,28,14,18,27,22,31,39,,,,,1.7,1.0,1.3*35
1    = Mode:
       M=Manual, forced to operate in 2D or 3D
       A=Automatic, 3D/2D
2    = Mode:
       1=Fix not available
       2=2D
       3=3D
3-14 = IDs of SVs used in position fix (null for unused fields)
15   = PDOP
16   = HDOP
17   = VDOP
*/

/*
$GPGSV - GPS Satellites in view
-------------------------------
eg. $GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74
    $GPGSV,3,2,11,14,25,170,00,16,57,208,39,18,67,296,40,19,40,246,00*74
    $GPGSV,3,3,11,22,42,067,42,24,14,311,43,27,05,244,00,,,,*4D

    $GPGSV,1,1,13,02,02,213,,03,-3,000,,11,00,121,,14,13,172,05*67

1    = Total number of messages of this type in this cycle
2    = Message number
3    = Total number of SVs in view
4    = SV PRN number
5    = Elevation in degrees, 90 maximum
6    = Azimuth, degrees from true north, 000 to 359
7    = SNR, 00-99 dB (null when not tracking)
8-11 = Information about second SV, same as field 4-7
12-15= Information about third SV, same as field 4-7
16-19= Information about fourth SV, same as field 4-7
*/

/*
$GPRMC - Recommended minimum specific GPS/Transit data
------------------------------------------------------
eg1. $GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62
eg2. $GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68

           225446       Time of fix 22:54:46 UTC
           A            Navigation receiver warning A = OK, V = warning
           4916.45,N    Latitude 49 deg. 16.45 min North
           12311.12,W   Longitude 123 deg. 11.12 min West
           000.5        Speed over ground, Knots
           054.7        Course Made Good, True
           191194       Date of fix  19 November 1994
           020.3,E      Magnetic variation 20.3 deg East
           *68          mandatory checksum

eg3. $GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*70
              1    2    3    4    5     6    7    8      9     10  11 12

      1   220516     Time Stamp
      2   A          validity - A-ok, V-invalid
      3   5133.82    current Latitude
      4   N          North/South
      5   00042.24   current Longitude
      6   W          East/West
      7   173.8      Speed in knots
      8   231.8      True course
      9   130694     Date Stamp
      10  004.2      Variation
      11  W          East/West
      12  *70        checksum

eg4. $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
1    = UTC of position fix
2    = Data status (V=navigation receiver warning)
3    = Latitude of fix
4    = N or S
5    = Longitude of fix
6    = E or W
7    = Speed over ground in knots
8    = Track made good in degrees True
9    = UT date
10   = Magnetic variation degrees (Easterly var. subtracts from true course)
11   = E or W
12   = Checksum
*/

/*
$GPVTG - Track Made Good and Ground Speed
-----------------------------------------
eg1. $GPVTG,360.0,T,348.7,M,000.0,N,000.0,K*43
eg2. $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K

           054.7,T      True track made good
           034.4,M      Magnetic track made good
           005.5,N      Ground speed, knots
           010.2,K      Ground speed, Kilometers per hour

eg3. $GPVTG,t,T,,,s.ss,N,s.ss,K*hh
1    = Track made good
2    = Fixed text 'T' indicates that track made good is relative to true north
3    = not used
4    = not used
5    = Speed over ground in knots
6    = Fixed text 'N' indicates that speed over ground in in knots
7    = Speed over ground in kilometers/hour
8    = Fixed text 'K' indicates that speed over ground is in kilometers/hour
9    = Checksum

The actual track made good and speed relative to the ground.

$--VTG,x.x,T,x.x,M,x.x,N,x.x,K
x.x,T = Track, degrees True
x.x,M = Track, degrees Magnetic
x.x,N = Speed, knots
x.x,K = Speed, Km/hr 
*/

/*
-------------------------------------------------------------------------------------------
https://web.archive.org/web/20131021232453/http://edu-observatory.org/gps/gps_accuracy.html

GPS Errors & Estimating Your Receiver's Accuracy

Accuracy: The degree of conformance between the estimated or measured position, time, and/or velocity 
of a GPS receiver and its true time, position, and/or velocity as compared with a constant standard. 

Estimated Position Error (EPE) and Error Sources (UERE=User Equivalent Range Error):

EPE (1-sigma) = HDOP * UERE (1-sigma)                 (1)

Multiplying the HDOP * UERE * 2 gives EPE (2drms) and is commonly taken as the 95% limit for the magnitude 
of the horizontal error. The probability of horizontal error is within an ellipse of radius 2drms ranges 
between 0.95 and 0.98 depending on the ratio of the ellipse semi-axes. User Equivalent Range Error (UERE) 
is computed in the tables lower on this page.

EPE (2drms) = 2 * HDOP * SQRT [URE^2 + UEE^2]         (2)

HDOP (Horizontal Geometric Dilution of Precision), GDOP, PDOP and VDOP are determined by the geometry of the 
current satellites visible above the receiver's mask angle with respect to user receiver's antenna. DOPs can 
be degraded (made larger) by signal obstruction due to terrain, foliage, building, vehicle structure, etc.

URE (User Range Error) is an estimate of "Signals in Space" errors, i.e., ephemeris data, satellite clocks, 
ionospheric delay and tropospheric delay. These errors can be greatly reduced by differential and multiple 
frequency techniques. Differential correction sources include user provided reference stations, community base 
stations, governmental beacon transmissions, FM sub-carrier transmissions and geosynchronous satellite 
transmissions.

UEE (User Equipment Errors) includes receiver noise, multipath, antenna orientation, EMI/RFI. Receiver and 
antenna design can greatly reduce UEE error sources--usually at substantial cost.

Position error can range from tens of meters (recreational) to a few millimeters (survey) depending on equipment, 
signals and usage. Professional mapping and survey equipment often includes user-settable minimum thresholds for 
SNR, mask angle, DOP, number of SVs used, etc. 

Table - Standard error model - L1 C/A (no SA)
                                One-sigma error, meters
Error source          Bias  Random  Total   DGPS
------------------------------------------------------------
Ephemeris data      2.1   0.0         2.1 0.0
Satellite clock           2.0   0.7         2.1     0.0
Ionosphere            4.0   0.5         4.0     0.4
Troposphere       0.5   0.5         0.7     0.2
Multipath             1.0   1.0         1.4     1.4
Receiver measurement    0.5   0.2         0.5     0.5
------------------------------------------------------------
User equivalent range 
  error (UERE), rms*    5.1   1.4         5.3     1.6
Filtered UERE, rms    5.1   0.4         5.1     1.5
------------------------------------------------------------
Vertical one-sigma errors--VDOP= 2.5           12.8     3.9
Horizontal one-sigma errors--HDOP= 2.0         10.2     3.1

Table - Standard error model - L1 C/A (with SA)
                                One-sigma error, meters
Error source          Bias  Random  Total   DGPS
------------------------------------------------------------
Ephemeris data      2.1   0.0         2.1   0.0
Satellite clock (dither)      20.0  0.7         20.0    0.0
Ionosphere            4.0   0.5         4.0     0.4
Troposphere       0.5   0.5         0.7     0.2
Multipath             1.0   1.0         1.4     1.4
Receiver measurement    0.5   0.2     0.5     0.5
------------------------------------------------------------
User equivalent range 
  error (UERE), rms*        20.5  1.4         20.6     1.6
Filtered UERE, rms        20.5  0.4         20.5     1.5
------------------------------------------------------------
Vertical one-sigma errors--VDOP= 2.5            51.4     3.9
Horizontal one-sigma errors--HDOP= 2.0          41.1     3.1


Table - Precise error model, dual-frequency, P(Y) code
                               One-sigma error, meters
Error source          Bias  Random  Total   DGPS
------------------------------------------------------------
Ephemeris data      2.1   0.0         2.1 0.0
Satellite clock           2.0   0.7         2.1     0.0
Ionosphere            1.0   0.5         1.2     0.1
Troposphere       0.5   0.5         0.7     0.1
Multipath             1.0   1.0         1.4     1.4
Receiver measurement    0.5   0.2         0.5     0.5
-----------------------------------------------------------
User equivalent range 
  error (UERE), rms*    3.3   1.5         3.6     1.5
Filtered UERE, rms    3.3   0.4         3.3     1.4
-----------------------------------------------------------
Vertical one-sigma errors--VDOP= 2.5            8.3     3.7
Horizontal one-sigma errors--HDOP= 2.0          6.6     3.0


Probabilidad      de estar a una distancia
68,0%             1*(VDOP * UERE)
95,4%             2*(VDOP * UERE)
99,7%             3*(VDOP * UERE)

DOP
0-1 IDEAL
1-2 EXCELLENT
2-5 GOOD
5-10 MODERATE
10-20 FAIR
>20 POOR

-------------------------------------------------------------------------------------------
*/
