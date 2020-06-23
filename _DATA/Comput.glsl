#version 430

//#extension GL_ARB_compute_variable_group_size : enable

//layout( local_size_variable ) in;
  layout( local_size_x = 10,
          local_size_y = 10,
          local_size_z =  1 ) in;

////////////////////////////////////////////////////////////////////////////////

  ivec3 _WorkGrupsN = ivec3( gl_NumWorkGroups );

//ivec3 _WorkItemsN = ivec3( gl_LocalGroupSizeARB );
  ivec3 _WorkItemsN = ivec3( gl_WorkGroupSize     );

  ivec3 _WorksN     = _WorkGrupsN * _WorkItemsN;

  ivec3 _WorkID     = ivec3( gl_GlobalInvocationID );

//############################################################################## ■

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$【定数】

const float Pi        = 3.141592653589793;
const float Pi2       = Pi * 2.0;
const float P2i       = Pi / 2.0;
const float FLOAT_MAX = 3.402823e+38;

const float SumX = 2285.826937;                                                 // Iin(λ)*x-(λ)のsum
const vec2[81] ArrayX = vec2[](                                                 // vec2(λ, Iin(λ)*x-(λ))の配列
        vec2(380,0.00528),
        vec2(385,0.02642704),
        vec2(390,0.1119588),
        vec2(395,0.39954114),
        vec2(400,1.209663),
        vec2(405,3.116554),
        vec2(410,6.8297216),
        vec2(415,12.59132014),
        vec2(420,20.0606652),
        vec2(425,28.0091746),
        vec2(430,35.3699196),
        vec2(435,42.12141225),
        vec2(440,46.623681),
        vec2(445,47.7413247),
        vec2(450,45.967048),
        vec2(455,42.3894852),
        vec2(460,37.2098063),
        vec2(465,31.3286805),
        vec2(470,24.2175084),
        vec2(475,16.42318741),
        vec2(480,9.9748173),
        vec2(485,5.04857024),
        vec2(490,1.9519604),
        vec2(495,0.5999308),
        vec2(500,0.4277736),
        vec2(505,1.65219912),
        vec2(510,3.8326695),
        vec2(515,7.05088398),
        vec2(520,11.4098781),
        vec2(525,16.73839134),
        vec2(530,23.176118),
        vec2(535,30.40304722),
        vec2(540,38.4684212),
        vec2(545,46.9421568),
        vec2(550,55.7376952),
        vec2(555,65.09832051),
        vec2(560,74.2600872),
        vec2(565,82.64584952),
        vec2(570,89.8864065),
        vec2(575,95.2588743),
        vec2(580,99.184848),
        vec2(585,102.520449),
        vec2(590,104.246064),
        vec2(595,103.470846),
        vec2(600,100.821903),
        vec2(605,96.744753),
        vec2(610,91.094432),
        vec2(615,83.8457606),
        vec2(620,75.4397657),
        vec2(625,66.4791358),
        vec2(630,56.977096),
        vec2(635,47.0147646),
        vec2(640,37.8915826),
        vec2(645,30.2412831),
        vec2(650,23.6666178),
        vec2(655,18.01926),
        vec2(660,13.4107272),
        vec2(665,9.7869562),
        vec2(670,7.0128243),
        vec2(675,4.941429),
        vec2(680,3.431484),
        vec2(685,2.35309683),
        vec2(690,1.5992682),
        vec2(695,1.08299808),
        vec2(700,0.7307251),
        vec2(705,0.4911478),
        vec2(710,0.3296372),
        vec2(715,0.221408),
        vec2(720,0.1485525),
        vec2(725,0.0998478),
        vec2(730,0.067298),
        vec2(735,0.0456556),
        vec2(740,0.031242),
        vec2(745,0.0214312),
        vec2(750,0.0148592),
        vec2(755,0.010413),
        vec2(760,0.0073206),
        vec2(765,0.00522),
        vec2(770,0.003783),
        vec2(775,0.002691),
        vec2(780,0.0019503)
);

const float SumY = 2349.618583;
const vec2[81] ArrayY = vec2[](
        vec2(380,0.000561),
        vec2(385,0.00287424),
        vec2(390,0.0119922),
        vec2(395,0.04242573),
        vec2(400,0.1268532),
        vec2(405,0.32379129),
        vec2(410,0.7057336),
        vec2(415,1.29424568),
        vec2(420,2.0984571),
        vec2(425,3.1207826),
        vec2(430,4.3471824),
        vec2(435,5.8406355),
        vec2(440,7.5423555),
        vec2(445,9.2222088),
        vec2(450,11.092544),
        vec2(455,13.1332416),
        vec2(460,15.7815431),
        vec2(465,18.8354313),
        vec2(470,22.926522),
        vec2(475,27.2923546),
        vec2(480,31.4196771),
        vec2(485,36.5889818),
        vec2(490,40.9333531),
        vec2(495,46.2198051),
        vec2(500,51.6531017),
        vec2(505,56.8448928),
        vec2(510,62.0696043),
        vec2(515,67.7500646),
        vec2(520,73.8142533),
        vec2(525,79.6818774),
        vec2(530,85.770678),
        vec2(535,92.3255714),
        vec2(540,98.2189748),
        vec2(545,102.09969),
        vec2(550,104.3332572),
        vec2(555,105.5759537),
        vec2(560,105.019902),
        vec2(565,102.2755818),
        vec2(570,97.7529696),
        vec2(575,91.65477625),
        vec2(580,84.9817452),
        vec2(585,78.78920289),
        vec2(590,72.454146),
        vec2(595,65.71060066),
        vec2(600,59.0531877),
        vec2(605,52.75418274),
        vec2(610,46.6719292),
        vec2(615,40.72914046),
        vec2(620,35.0688217),
        vec2(625,29.90112524),
        vec2(630,24.947384),
        vec2(635,20.05439644),
        vec2(640,15.7888984),
        vec2(645,12.33716589),
        vec2(650,9.4932306),
        vec2(655,7.1606934),
        vec2(660,5.2986999),
        vec2(665,3.84605312),
        vec2(670,2.74434),
        vec2(675,1.9279506),
        vec2(680,1.33602),
        vec2(685,0.9149973),
        vec2(690,0.6214698),
        vec2(695,0.42054),
        vec2(700,0.2836834),
        vec2(705,0.1907334),
        vec2(710,0.1280032),
        vec2(715,0.0860288),
        vec2(720,0.0577818),
        vec2(725,0.0388518),
        vec2(730,0.0262108),
        vec2(735,0.0178352),
        vec2(740,0.0122385),
        vec2(745,0.008428),
        vec2(750,0.0058016),
        vec2(755,0.004095),
        vec2(760,0.002905),
        vec2(765,0.002088),
        vec2(770,0.001455),
        vec2(775,0.001053),
        vec2(780,0.0007683)
);

const float SumZ = 2728.954066;
const vec2[81] ArrayZ = vec2[](
        vec2(380,0.023265),
        vec2(385,0.11688576),
        vec2(390,0.4968468),
        vec2(395,1.78441848),
        vec2(400,5.4444963),
        vec2(405,14.1551872),
        vec2(410,31.3828996),
        vec2(415,58.7997228),
        vec2(420,95.4063702),
        vec2(425,135.6885),
        vec2(430,174.611152),
        vec2(435,211.773375),
        vec2(440,239.02452),
        vec2(445,250.270185),
        vec2(450,247.3552),
        vec2(455,234.92652),
        vec2(460,214.855047),
        vec2(465,191.71917),
        vec2(470,163.113928),
        vec2(475,127.837518),
        vec2(480,95.6662875),
        vec2(485,70.0717752),
        vec2(490,50.1211578),
        vec2(495,35.3454164),
        vec2(500,24.4940742),
        vec2(505,17.03645802),
        vec2(510,11.4621012),
        vec2(515,8.12692488),
        vec2(520,5.8827021),
        vec2(525,4.166379),
        vec2(530,2.984198),
        vec2(535,2.05716496),
        vec2(540,1.3963196),
        vec2(545,0.8230761),
        vec2(550,0.4195376),
        vec2(555,0.11528597),
        vec2(560,0),
        vec2(565,0),
        vec2(570,0),
        vec2(575,0),
        vec2(580,0),
        vec2(585,0),
        vec2(590,0),
        vec2(595,0),
        vec2(600,0),
        vec2(605,0),
        vec2(610,0),
        vec2(615,0),
        vec2(620,0),
        vec2(625,0),
        vec2(630,0),
        vec2(635,0),
        vec2(640,0),
        vec2(645,0),
        vec2(650,0),
        vec2(655,0),
        vec2(660,0),
        vec2(665,0),
        vec2(670,0),
        vec2(675,0),
        vec2(680,0),
        vec2(685,0),
        vec2(690,0),
        vec2(695,0),
        vec2(700,0),
        vec2(705,0),
        vec2(710,0),
        vec2(715,0),
        vec2(720,0),
        vec2(725,0),
        vec2(730,0),
        vec2(735,0),
        vec2(740,0),
        vec2(745,0),
        vec2(750,0),
        vec2(755,0),
        vec2(760,0),
        vec2(765,0),
        vec2(770,0),
        vec2(775,0),
        vec2(780,0)
);

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$【ルーチン】

float Pow2( in float X )
{
  return X * X;
}

//------------------------------------------------------------------------------

float length2( in vec3 V )
{
  return Pow2( V.x ) + Pow2( V.y ) + Pow2( V.z );
}

//------------------------------------------------------------------------------

int MinI( float A_, float B_, float C_ )
{
    if ( A_ <= B_ )
    {
        if ( A_ <= C_ ) return 0;
                   else return 2;
    }
    else
    {
        if ( B_ <= C_ ) return 1;
                   else return 2;
    }
}

int MinI( vec3 V_ )
{
    return MinI( V_.x, V_.y, V_.z );
}

//------------------------------------------------------------------------------

vec2 VecToSky( in vec3 Vec )
{
  vec2 Result;

  Result.x = ( Pi - atan( -Vec.x, -Vec.z ) ) / Pi2;
  Result.y =        acos(  Vec.y           ) / Pi ;

  return Result;
}

//------------------------------------------------------------------------------

vec3 ToneMap( in vec3 Color, in float White )
{
  return clamp( Color * ( 1 + Color / White ) / ( 1 + Color ), 0, 1 );
}

//------------------------------------------------------------------------------

vec3 GammaCorrect( in vec3 Color, in float Gamma )
{
  vec3 Result;

  float G = 1 / Gamma;

  Result.r = pow( Color.r, G );
  Result.g = pow( Color.g, G );
  Result.b = pow( Color.b, G );

  return Result;
}

//------------------------------------------------------------------------------

float Fresnel( in vec3 Vec, in vec3 Nor, in float IOR )
{
  // float N = Pow2( IOR );
  // float C = dot( Vec, Nor );
  // float G = sqrt( N + Pow2( C ) - 1 );
  // float NC = N * C;
  // return ( Pow2( (  C + G ) / (  C - G ) )
  //        + Pow2( ( NC + G ) / ( NC - G ) ) ) / 2;

  float R = Pow2( ( IOR - 1 ) / ( IOR + 1 ) );
  float C = clamp( dot( Vec, Nor ), -1, 0 );
  return R + ( 1 - R ) * pow( 1 + C, 5 );
}

//------------------------------------------------------------------------------

uvec4 _RandSeed;

uint rotl( in uint x, in int k )
{
  return ( x << k ) | ( x >> ( 32 - k ) );
}

float Rand()
{
  const uint Result = rotl( _RandSeed[ 0 ] * 5, 7 ) * 9;

  const uint t = _RandSeed[ 1 ] << 9;

  _RandSeed[ 2 ] ^= _RandSeed[ 0 ];
  _RandSeed[ 3 ] ^= _RandSeed[ 1 ];
  _RandSeed[ 1 ] ^= _RandSeed[ 2 ];
  _RandSeed[ 0 ] ^= _RandSeed[ 3 ];

  _RandSeed[ 2 ] ^= t;

  _RandSeed[ 3 ] = rotl( _RandSeed[ 3 ], 11 );

  return float( Result ) / 4294967296.0;
}

//------------------------------------------------------------------------------

vec2 RandCirc()
{
  vec2 Result;
  float T, R;

  T = Pi2 * Rand();
  R = sqrt( Rand() );

  Result.x = R * cos( T );
  Result.y = R * sin( T );

  return Result;
}

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$【外部変数】

layout( rgba32ui ) uniform uimage2D _Seeder;

layout( std430 ) buffer TAccumN
{
  int _AccumN;
};

layout( rgba32f ) uniform image2D _Accumr;

writeonly uniform image2D _Imager;

layout( std430 ) buffer TCamera
{
  layout( row_major ) mat4 _Camera;
};

uniform sampler2D _Textur;

layout( rgba32f ) uniform image3D _Voxels;

//############################################################################## ■

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TRay

struct TRay
{
  vec4 Pos;
  vec4 Vec;
  float Wei;
  float Emi;
  int Wav;
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% THit

struct THit
{
  float t;
  int   Mat;
  vec4  Pos;
  vec4  Nor;
};

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$【内部変数】

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$【物体】

mat4 _ObjMove = mat4( 1 );
mat4 _ObjMovi = mat4( 1 );

void BeginMove( inout TRay Ray )
{
  _ObjMovi = inverse( _ObjMove );

  Ray.Pos = _ObjMovi * Ray.Pos;
  Ray.Vec = _ObjMovi * Ray.Vec;
}

void EndMove( inout THit Hit )
{
  mat3 ObjMovn = transpose( mat3( _ObjMovi ) );

  Hit.Pos     = _ObjMove * Hit.Pos    ;
  Hit.Nor.xyz =  ObjMovn * Hit.Nor.xyz;
}

////////////////////////////////////////////////////////////////////////////////

void ObjPlane( in TRay Ray, inout THit Hit )
{
  float t;

  if ( Ray.Vec.y < 0 )
  {
    t = ( Ray.Pos.y - -1.001 ) / -Ray.Vec.y;

    if ( ( 0 < t ) && ( t < Hit.t ) )
    {
      Hit.t   = t;
      Hit.Pos = Ray.Pos + t * Ray.Vec;
      Hit.Nor = vec4( 0, 1, 0, 0 );
      Hit.Mat = 3;
    }
  }
}

//------------------------------------------------------------------------------

bool ObjSpher( in TRay Ray, inout THit Hit )
{
  BeginMove( Ray );

  float L, B, C, D, t;

  L = length( Ray.Vec.xyz );

  B = dot( Ray.Pos.xyz, Ray.Vec.xyz / L );
  C = length2( Ray.Pos.xyz ) - 1;

  D = Pow2( B ) - C;

  if ( D > 0 )
  {
    t = ( -B - sign( C ) * sqrt( D ) ) / L;

    if ( ( 0 < t ) && ( t < Hit.t ) )
    {
      Hit.t   = t;
      Hit.Pos = Ray.Pos + t * Ray.Vec;
      Hit.Nor = Hit.Pos;
      Hit.Mat = 7;

      EndMove( Hit );

      return true;
    }
  }

  return false;
}

//------------------------------------------------------------------------------

bool Slab( in float Pos, in float Vec,
           in float Cen, in float Siz,
           inout float MinT, inout float MaxT )
{
  float S, T0, T1;

  S = sign( Vec );

  T0 = ( Cen - S * Siz/2 - Pos ) / Vec;
  T1 = ( Cen + S * Siz/2 - Pos ) / Vec;

  if ( T1 < MaxT ) { MaxT = T1; }

  if ( MinT < T0 ) { MinT = T0; return true; } else { return false; }
}

void ObjRecta( in TRay Ray, inout THit Hit, in vec3 Cen, in vec3 Siz )
{
  float MinT, MaxT;
  vec3 Nor;

  MinT = -FLOAT_MAX;
  MaxT = +FLOAT_MAX;

  if ( Slab( Ray.Pos.x, Ray.Vec.x, Cen.x, Siz.x, MinT, MaxT ) ) Nor = vec3( -sign( Ray.Vec.x ), 0, 0 );
  if ( Slab( Ray.Pos.y, Ray.Vec.y, Cen.y, Siz.y, MinT, MaxT ) ) Nor = vec3( 0, -sign( Ray.Vec.y ), 0 );
  if ( Slab( Ray.Pos.z, Ray.Vec.z, Cen.z, Siz.z, MinT, MaxT ) ) Nor = vec3( 0, 0, -sign( Ray.Vec.z ) );

  if( ( MinT < MaxT ) && ( 0 < MinT ) && ( MinT < Hit.t ) )
  {
    Hit.t   = MinT;
    Hit.Pos = Ray.Pos + MinT * Ray.Vec;
    Hit.Nor = vec4( Nor, 0 );
    Hit.Mat = 1;
  }
}

bool HitRecta( in TRay Ray, out float HitT, in vec3 Cen, in vec3 Siz )
{
  float MinT, MaxT;

  MinT = -FLOAT_MAX;
  MaxT = +FLOAT_MAX;

  Slab( Ray.Pos.x, Ray.Vec.x, Cen.x, Siz.x, MinT, MaxT );
  Slab( Ray.Pos.y, Ray.Vec.y, Cen.y, Siz.y, MinT, MaxT );
  Slab( Ray.Pos.z, Ray.Vec.z, Cen.z, Siz.z, MinT, MaxT );

  if( MinT < MaxT ) { HitT = MinT; return true; } else return false;
}

//------------------------------------------------------------------------------

const vec3  _GridsC =  vec3(  0,  0,  0 );
const vec3  _GridsS =  vec3(  2,  2,  2 );
const ivec3 _GridsN = ivec3( 10, 10, 10 );

void ObjGrids( in TRay Ray, inout THit Hit )
{
  float HitT;

  if ( HitRecta( Ray, HitT, _GridsC, 0.9999 * _GridsS ) )
  {
    vec4 HitP = max( HitT, 0 ) * Ray.Vec + Ray.Pos;

    ivec3 Gv = ivec3( sign( Ray.Vec.xyz ) );

    ivec3 Gvs[ 3 ] = { { Gv.x,    0,    0 },
                       {    0, Gv.y,    0 },
                       {    0,    0, Gv.z } };

    vec3 Sd = _GridsS / _GridsN;

    vec3 Tv = Sd / abs( Ray.Vec.xyz );

    vec3 Tvs[ 3 ] = { { Tv.x,    0,    0 },
                      {    0, Tv.y,    0 },
                      {    0,    0, Tv.z } };

    vec3 G = ( HitP.xyz - _GridsC + _GridsS / 2 ) / Sd;

    ivec3 Gi = ivec3( floor( G ) );

    vec3 Gd = G - Gi;

    vec3 Ts;

    if ( isinf( Tv.x ) ) Ts.x = FLOAT_MAX;
                    else Ts.x = Tv.x * ( 0.5 + sign( Ray.Vec.x ) * ( 0.5 - Gd.x ) );

    if ( isinf( Tv.y ) ) Ts.y = FLOAT_MAX;
                    else Ts.y = Tv.y * ( 0.5 + sign( Ray.Vec.y ) * ( 0.5 - Gd.y ) );

    if ( isinf( Tv.z ) ) Ts.z = FLOAT_MAX;
                    else Ts.z = Tv.z * ( 0.5 + sign( Ray.Vec.z ) * ( 0.5 - Gd.z ) );

    float T0 = 0;

    while ( ( 0 <= Gi.x ) && ( Gi.x < _GridsN.x )
         && ( 0 <= Gi.y ) && ( Gi.y < _GridsN.y )
         && ( 0 <= Gi.z ) && ( Gi.z < _GridsN.z ) )
    {
      int K = MinI( Ts );

      float T1 = Ts[ K ];

      vec3 R = Sd / 2 * imageLoad( _Voxels, Gi ).rgb;
      vec3 C = _GridsC + _GridsS * ( ( Gi + 0.5 ) / _GridsN - vec3( 0.5 ) );

      _ObjMove = mat4( R.x,    0,    0,  0,
                         0,  R.y,    0,  0,
                         0,    0,  R.z,  0,
                       C.x,  C.y,  C.z,  1 );

      if ( ObjSpher( Ray, Hit ) ) break;

      T0 = T1;

      Gi += Gvs[ K ];
      Ts += Tvs[ K ];
    }
  }
}

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$【材質】

float _EmitShift = 0.0001;

////////////////////////////////////////////////////////////////////////////////

TRay MatSkyer( in TRay Ray, in THit Hit )
{
  TRay Result;

  Result.Vec = Ray.Vec;
  Result.Pos = Ray.Pos;
  Result.Wei = Ray.Wei;
  Result.Wav = Ray.Wav;

  switch ( Ray.Wav )                                                            // 波長ごとにRGBそれぞれの輝度を返す
  {
    case 700: Result.Emi =  texture( _Textur, VecToSky( Ray.Vec.xyz ) ).r; break;
    case 546: Result.Emi =  texture( _Textur, VecToSky( Ray.Vec.xyz ) ).g; break;
    case 436: Result.Emi =  texture( _Textur, VecToSky( Ray.Vec.xyz ) ).b; break;
  }

  return Result;
}

//------------------------------------------------------------------------------

TRay MatMirro( in TRay Ray, in THit Hit )
{
  TRay Result;

  Result.Vec = vec4( reflect( Ray.Vec.xyz, Hit.Nor.xyz ), 0 );
  Result.Pos = Hit.Pos + _EmitShift * Hit.Nor;
  Result.Wei = Ray.Wei;
  Result.Emi = Ray.Emi;

  return Result;
}

//------------------------------------------------------------------------------

TRay MatWater( inout TRay Ray, in THit Hit )
{
  TRay Result;
  float IOR, F;
  vec4  Nor;

  if( dot( Ray.Vec.xyz, Hit.Nor.xyz ) < 0 )
  {
    IOR = 1.333 / 1.000;
    Nor = +Hit.Nor;
  }
  else
  {
    IOR = 1.000 / 1.333;
    Nor = -Hit.Nor;
  }

  F = Fresnel( Ray.Vec.xyz, Nor.xyz, IOR );

  if ( Rand() < F )
  {
    Result.Vec = vec4( reflect( Ray.Vec.xyz, Nor.xyz ), 0 );
    Result.Pos = Hit.Pos + _EmitShift * Nor;
    Result.Wei = Ray.Wei;
    Result.Emi = Ray.Emi;
  } else {
    Result.Vec = vec4( refract( Ray.Vec.xyz, Nor.xyz, 1 / IOR ), 0 );
    Result.Pos = Hit.Pos - _EmitShift * Nor;
    Result.Wei = Ray.Wei;
    Result.Emi = Ray.Emi;
  }

  return Result;
}

//------------------------------------------------------------------------------

TRay MatDiffu( in TRay Ray, in THit Hit )
{
  TRay Result;

  Result.Vec.y = sqrt( Rand() );

  float d = sqrt( 1 - Pow2( Result.Vec.y ) );
  float v = Rand();

  Result.Vec.x = d * cos( Pi2 * v );
  Result.Vec.z = d * sin( Pi2 * v );

  Result.Pos = Hit.Pos + _EmitShift * Hit.Nor;
  Result.Wei = Ray.Wei;
  Result.Emi = Ray.Emi;

  return Result;
}

//------------------------------------------------------------------------------

/********************************************************************************
TRay MatThinf( in TRay Ray, in THit Hit )
{
  TRay Result;
  float IOR1, IOR2, d, V, A, Rp, Rs, diff, Theta_i, Theta_o, Theta_t, X, Y, Z;                                            // Dとlambdaは、D nm、lambda nmとする
  float r, g, b;
  int i, count;

  IOR1 = 1.0;                                                                   // 空気と水を想定
  IOR2 = 1.33333;
  d = 1000 * Rand();                                                            // 薄膜の厚さ

  vec4 LIGHT = vec4(0, 2, 0, 0);                                                // 光源を設定

  Theta_i = acos( dot( Hit.Nor.xyz, normalize( LIGHT.xyz - Hit.Nor.xyz ) ) );   // 垂線と光源方向へのベクトルのなす角
  Theta_o = acos( dot( Hit.Nor.xyz, Ray.Vec.xyz ) );                            // 垂線と視点方向へのベクトルのなす角
  Theta_t = asin( IOR1 * sin( Theta_o ) / IOR2 );                               // Theta_oの屈折角

  Rp = Pow2( tan( Theta_i - Theta_t ) ) / Pow2( tan( Theta_i + Theta_t ) );
  Rs = Pow2( sin( Theta_i - Theta_t ) ) / Pow2( sin( Theta_i + Theta_t ) );
  A  = ( Rp + Rs ) / 2;
                                                                                // diff: 光路差
  diff = d * ( Pow2(IOR2) - 2 * Pow2(IOR1 * sin(Theta_i)) ) / sqrt(Pow2(IOR2) - Pow2(IOR1 * sin(Theta_i)) ) + d * Pow2(IOR2) / sqrt(Pow2(IOR2) - Pow2(IOR1 * sin(Theta_o)));

  V = 0;
  for( count = 0; count < 3; count++)
  {
    for( i = 0; i < 81; i++ )
    {
      V = V + cos( Pi2 * diff / ArrayX[i].x ) * ArrayX[i].y;
    }

    switch( count )
    {
      case 0: X = SumX * 2 * Pow2( A ) * Pow2( A ) * V; break;
      case 1: Y = SumY * 2 * Pow2( A ) * Pow2( A ) * V; break;
      case 2: Z = SumZ * 2 * Pow2( A ) * Pow2( A ) * V; break;
    }
  }

  r =  3.240479 * X - 1.537150 * Y - 0.498535 * Z;
  g = -0.969256 * X + 1.875992 * Y + 0.041556 * Z;
  b =  0.055648 * X - 0.204043 * Y + 1.057311 * Z;

  Result.Vec = vec4( reflect( Ray.Vec.xyz, Hit.Nor.xyz ), 0 );
  Result.Pos = Hit.Pos + _EmitShift * Hit.Nor;
  Result.Wei = Ray.Wei;
  Result.Emi = Ray.Emi + vec3( r, g, b );

  return Result;
}
*******************************************************************************/

//------------------------------------------------------------------------------


TRay MatThin2( in TRay Ray, in THit Hit )                                       // シャボン玉
{
  TRay Result;
  float IOR, F;
  vec4 Nor;

  if( dot( Ray.Vec.xyz, Hit.Nor.xyz ) < 0 )                                     // 以下、MatWaterベースでResult.VecとResult.Posを決める
  {                                                                             // 変更点：本来屈折が起こる時に、透過させるようにした
    IOR = 1.333 / 1.000;
    Nor = +Hit.Nor;
  }
  else
  {
    IOR = 1.000 / 1.333;
    Nor = -Hit.Nor;
  }

  float d = 700;                                                                // 膜の厚さ(nm)
  float IOR1 = 1.0;
  float IOR2 = 1.33333;

  float Theta_1 = acos( clamp( dot( Hit.Nor.xyz, -Ray.Vec.xyz ), -1, +1 ) );    // 視点方向からの光線の入射角 clampで-1~+1に制限
  float Theta_2 = asin( sin( Theta_1 ) * IOR1 / IOR2 );                         // 視点方向からの光線の屈折角

  float pm = d / cos( Theta_2 );
  float ps = 2 * d * sin( Theta_1 ) * tan( Theta_2 );
  float D  = 2 * IOR2 * pm - IOR1 * ps;                                         // 光路差

  float PD = Pi2 * mod( D / Ray.Wav, 1 );                                       // 位相差 Ray.Wav(nm)

  if ( Rand() < Pow2( cos( PD / 2 ) ) )
  {                                                                             // 反射時
    Result.Vec = vec4( reflect( Ray.Vec.xyz, Nor.xyz ), 0 );
    Result.Pos = Hit.Pos + _EmitShift * Nor;
  } else {                                                                      // 屈折時
    Result.Vec = Ray.Vec;                                                       // refractではなく、そのまま透過させた
    Result.Pos = Hit.Pos - _EmitShift * Nor;
  }

  Result.Wei = Ray.Wei;
  Result.Emi = Ray.Emi;
  Result.Wav = Ray.Wav;

  /*                                                                            // test
  switch ( Ray.Wav )
  {
    case 700: Result.Wei = Ray.Wei * 0.381229 ; break;
    case 546: Result.Wei = Ray.Wei * 0.3921851; break;
    case 436: Result.Wei = Ray.Wei * 0.4399138; break;
  }
  */

  return Result;
}

//------------------------------------------------------------------------------

TRay MatOxide( in TRay Ray, in THit Hit )                                       // チタン酸化被膜の色表現
{                                                                               // ハーフベクトルベースの薄膜干渉モデル
  TRay Result;
  float IOR0, IOR1, IOR2;                                                       // 0:空気、1:酸化被膜、2:チタン
  float Theta_v, Theta_l, Theta_h, PhDiff, Theta_1, Theta_2;                    // 後述
  float r01, r12, R;                                                            // r01,r12:フレネル反射係数 R:反射率
  float d = 226.6;                                                               // 薄膜の厚さ（nm）：膜厚を変えれば色が変わる

  IOR0 = 1.000;
  switch( Ray.Wav )                                                             // IOR1は（とりあえず）目分量
  {                                                                             // IOR2は一応線形計算
    case 700: IOR1 = 2.498; IOR2 = 2.409; break;
    case 546: IOR1 = 2.563; IOR2 = 1.877; break;
    case 436: IOR1 = 2.748; IOR2 = 1.657; break;
  }

  Result.Vec = vec4( reflect( Ray.Vec.xyz, Hit.Nor.xyz ), 0 );                  // 鏡面反射（じゃないほうがいいかも？）

  Theta_v = acos( dot( Hit.Nor.xyz, -Ray.Vec.xyz ) );                           // 視点方向からの光線の入射角
  Theta_l = acos( dot( Hit.Nor.xyz, Result.Vec.xyz ) );                         // 本来の光源方向からの光線の入射角
  Theta_h = ( Theta_v + Theta_l ) / 2;                                          // 今回のモデルの光源方向からの光線の入射角
  Theta_1 = asin( sin( Theta_h ) * IOR0 / IOR1 );                               // 今回のモデルの光源方向からの光線の屈折角
  Theta_2 = asin( sin( Theta_1 ) * IOR1 / IOR2 );                               // 入射角θ'、屈折角θ2

  PhDiff = Pi2 * 2 * IOR1 * d * cos( Theta_1 ) / Ray.Wav;                       // 位相差＝2π*2ndcosθ/λ

  r01 = Fresnel( Ray.Vec.xyz, Hit.Nor.xyz, IOR1 / IOR0 );                       // 偏光なしver
  r12 = Fresnel( Ray.Vec.xyz, Hit.Nor.xyz, IOR2 / IOR1 );

  /********************
  r01 =   ( IOR1 * cos( Theta_h ) - IOR0 * cos( Theta_1 ) )                     // p偏光
        / ( IOR1 * cos( Theta_h ) - IOR0 * cos( Theta_1 ) );
  r12 =   ( IOR2 * cos( Theta_1 ) - IOR1 * cos( Theta_2 ) )                     // p偏光
        / ( IOR2 * cos( Theta_1 ) - IOR1 * cos( Theta_2 ) );
  ********************/

  R =   ( Pow2( r01 + r12 * cos( PhDiff ) ) + Pow2( r12 * sin( PhDiff ) ) )     // 反射率
      / ( Pow2( 1 + r12 * r01 * cos( PhDiff ) ) + Pow2( r12 * r01 * sin( PhDiff ) ) );

  Result.Pos = Hit.Pos + _EmitShift * Hit.Nor;
  Result.Wei = Ray.Wei * R;
  Result.Emi = Ray.Emi;
  Result.Wav = Ray.Wav;

  return Result;
}

//------------------------------------------------------------------------------

/*******************************************************************************

TRay MatDiff2( in TRay Ray, in THit Hit )
{
  TRay Result;

  Result.Vec.y = sqrt( Rand() );                                                // MatDiffuと同じ
                                                                                //
  float d = sqrt( 1 - Pow2( Result.Vec.y ) );                                   //
  float v = Rand();                                                             //
                                                                                //
  Result.Vec.x = d * cos( Pi2 * v );                                            //
  Result.Vec.z = d * sin( Pi2 * v );                                            // ここまで

  float C = dot( Hit.Nor.xyz, Result.Vec.xyz );

  Result.Pos = Hit.Pos + _EmitShift * Hit.Nor;
  Result.Emi = Ray.Emi;

  if( C > 0 )
  {
    Result.Wei = Ray.Wei * vec3( 0.9606217, 0.8449545, 0.4129899 );
  }
  else
  {
    Result.Wei = Ray.Wei;
  }

  return Result;
}
*******************************************************************************/

//------------------------------------------------------------------------------

/*******************************************************************************
vec3 LambReflect( in TRay Ray )                                                 // カワラバトの表現のやつ
{
  TRay Result;

  vec4 LIGHT = vec4(0, 2, 0, 0);                                                // 光源を設定

  float C =  dot( Ray.Pos.xyz, Ray.Vec.xyz );                                   // 法線ベクトルと光源方向ベクトルのなす角のcos
                                                                                // dotの1個目のRay.Pos.xyzはSphereの時だけ成立
  if( C > 0 )
  {                                                                             // ランバート反射
    float r =  0.9606217 * Ray.Emi.x * C;                                       // 光源のRGB値は(1.0, 1.0, 1.0)に設定
    float g =  0.8449545 * Ray.Emi.y * C;                                       // 先頭の数値は薄膜反射率シュミレータから得たもの
    float b =  0.4129899 * Ray.Emi.z * C;                                       // 左は空気、水、金Auの薄膜干渉の場合
                                                                                // https://www.filmetricsinc.jp/reflectance-calculator
    return vec3( r, g, b );
  }
  else
  {
    return Ray.Emi.xyz;
  }
}
********************************************************************************/



//##############################################################################

void Raytrace( inout TRay Ray )
{
  THit Hit;

  for ( int L = 1; L <= 5; L++ )
  {
    Hit = THit( FLOAT_MAX, 0, vec4( 0 ), vec4( 0 ) );

    ///// 物体

    ObjSpher( Ray, Hit );

    ///// 材質

    switch( Hit.Mat )
    {
      case 0: Ray = MatSkyer( Ray, Hit ); return;
      case 1: Ray = MatMirro( Ray, Hit ); break;
      case 2: Ray = MatWater( Ray, Hit ); break;
      case 3: Ray = MatDiffu( Ray, Hit ); break;
    //case 4: Ray = MatThinf( Ray, Hit ); break;
    //case 5: Ray = MatLambe( Ray, Hit ); break;
    //case 6: Ray = MatDiff2( Ray, Hit ); break;
      case 7: Ray = MatThin2( Ray, Hit ); break;
      case 8: Ray = MatOxide( Ray, Hit ); break;
    }
  }
}

vec3 waveLengthToRGB( in float lambda )
{
    //中心波長[nm]
    float r0 = 700.0; //赤色
    float g0 = 546.1; //緑色
    float b0 = 435.8; //青色
    float o0 = 605.0; //橙色
    float y0 = 580.0; //黄色
    float c0 = 490.0; //藍色
    float p0 = 400.0; //紫色

    //半値半幅
    float wR = 90;
    float wG = 80;
    float wB = 80;
    float wO = 60;
    float wY = 50;
    float wC = 50;
    float wP = 40;
    //強度
    float iR = 0.95;
    float iG = 0.74;
    float iB = 0.75;

    float iO = 0.4;
    float iY = 0.1;
    float iC = 0.3;
    float iP = 0.3;

    //正規分布の計算
    float r = iR * exp( - ( lambda - r0 ) * ( lambda - r0 ) / ( wR * wR )  );
    float g = iG * exp( - ( lambda - g0 ) * ( lambda - g0 ) / ( wG * wG )  );
    float b = iB * exp( - ( lambda - b0 ) * ( lambda - b0 ) / ( wB * wB )  );
    float o = iO * exp( - ( lambda - o0 ) * ( lambda - o0 ) / ( wO * wO )  );
    float y = iY * exp( - ( lambda - y0 ) * ( lambda - y0 ) / ( wY * wY )  );
    float c = iC * exp( - ( lambda - c0 ) * ( lambda - c0 ) / ( wC * wC )  );
    float p = iP * exp( - ( lambda - p0 ) * ( lambda - p0 ) / ( wP * wP )  );

    /*
    orange #ffa500  1: 0.715 : 0.230
    yellow #ffff00  1: 1     : 0
    cian   #00ff00  0: 1     : 1
    purple #804080  1: 0.5   : 1
    */

    r = r + o + y + p;
    g = g + o*0.715 + y*0.83 + c + p *0.50;
    b = b + o*0.23 + c + p;

    if( r > 1.0 ) r = 1.0;
    if( g > 1.0 ) g = 1.0;
    if( b > 1.0 ) b = 1.0;

    return vec3(r,g,b);
}

//------------------------------------------------------------------------------

void main()
{
  vec4 E, S;
  TRay RayR, RayG, RayB;                                                        // RayをRGBごとに3本生成
  vec3 A, C, P;

  _RandSeed = imageLoad( _Seeder, _WorkID.xy );

  if ( _AccumN == 0 ) A = vec3( 0 );
                 else A = imageLoad( _Accumr, _WorkID.xy ).rgb;

  for( uint N = _AccumN+1; N <= _AccumN+16; N++ )
  {
    E = vec4( 0, 0, 0, 1 );
    E.xy += 0.05 * RandCirc();

    S.x = 4.0 * ( _WorkID.x + 0.5 ) / _WorksN.x - 2.0;
    S.y = 1.5 - 3.0 * ( _WorkID.y + 0.5 ) / _WorksN.y;
    S.z = -2;
    S.w = 1;

    RayR.Pos = _Camera * E;
    RayR.Vec = _Camera * normalize( S - E );
    RayR.Wei = 1.0;
    RayR.Emi = 0;

    RayG.Pos = _Camera * E;
    RayG.Vec = _Camera * normalize( S - E );
    RayG.Wei = 1.0;
    RayG.Emi = 0;

    RayB.Pos = _Camera * E;
    RayB.Vec = _Camera * normalize( S - E );
    RayB.Wei = 1.0;
    RayB.Emi = 0;

    RayR.Wav = 700;
    RayG.Wav = 546;
    RayB.Wav = 436;

    /*
      switch( int( floor( Rand() * 3 ) ) )
    {
      case 0: R.Wav =  700;
      case 1: R.Wav =  546;
      case 2: R.Wav =  436;
    }
    //R.Wav = ( 780.0 - 380.0 ) * Rand() + 380;
    */

    Raytrace( RayR );
    Raytrace( RayG );
    Raytrace( RayB );

    C =   RayR.Wei * RayR.Emi * waveLengthToRGB( RayR.Wav )
        + RayG.Wei * RayG.Emi * waveLengthToRGB( RayG.Wav )
        + RayB.Wei * RayB.Emi * waveLengthToRGB( RayB.Wav );

    A += ( C - A ) / N;
  }

  imageStore( _Accumr, _WorkID.xy, vec4( A, 1 ) );

  P = GammaCorrect( ToneMap( A, 10 ), 2.2 );

  imageStore( _Imager, _WorkID.xy, vec4( P, 1 ) );

  imageStore( _Seeder, _WorkID.xy, _RandSeed );
}

//############################################################################## ■
