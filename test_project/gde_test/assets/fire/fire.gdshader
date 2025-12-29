shader_type spatial;
render_mode unshaded;

group_uniforms FlameShape;
uniform float top_radius : hint_range(0.0, 1.5);
uniform float bottom_radius : hint_range(0.0, 1.5);
uniform float center_radius : hint_range(0.0, 5.0);
uniform float center_position : hint_range(0.0, 1.0);
uniform float cylinder_height : hint_range(0.0, 10.0) = 1.0;
uniform float bottom_curve = 0.4;
uniform float _vibrate_amplitude = 0.015;
uniform float _vibrate_frequency = 200.0;
group_uniforms;

group_uniforms FlameAppearance;
uniform float _speed = 2.0;
uniform float _scale = 2.0;
group_uniforms;

varying float cylinderheight;

void vertex() {
	float topradius = top_radius;
	float bottomradius =  bottom_radius;
	float centerradius = center_radius;
	float centerposition = center_position;
	cylinderheight = cylinder_height;
	
    vec3 v_pos = VERTEX;
    float original_height = v_pos.y;  
    float relative_height = (original_height + 1.0) / 2.0;
	
	float scaledHeight =  relative_height * cylinderheight;
	v_pos.y = scaledHeight + sin(TIME * _vibrate_frequency) * _vibrate_amplitude;
   
    float radius;
    if (relative_height <= centerposition) {       
        float t = relative_height / centerposition;
		t = pow(t, bottom_curve);
        t = smoothstep(0.0, 1.0, t);
        radius = mix(bottom_radius, centerradius, t);
    } else {        
        float t = (relative_height - centerposition) / (1.0 - centerposition);
        t = smoothstep(0.0, 1.0, t);
        radius = mix(centerradius, topradius, t);
    }
    
    float current_radius = length(vec2(v_pos.x, v_pos.z));    
    if (current_radius > 0.0) {
        float scale_factor = radius / current_radius;
        v_pos.x *= scale_factor;
        v_pos.z *= scale_factor;
    }
    
    VERTEX = v_pos;	
}

vec3 mod2893(vec3 x) {
  return x - floor(x * (1.0 / 289.0)) * 289.0;
}

vec4 mod289(vec4 x) {
  return x - floor(x * (1.0 / 289.0)) * 289.0;
}

vec4 permute(vec4 x) {
     return mod289(((x*34.0)+1.0)*x);
}

vec4 taylorInvSqrt(vec4 r)
{
  return 1.79284291400159 - 0.85373472095314 * r;
}

float snoise(vec3 v)
  { 
  const vec2  C = vec2(1.0/6.0, 1.0/3.0) ;
  const vec4  D = vec4(0.0, 0.5, 1.0, 2.0);

// First corner
  vec3 i  = floor(v + dot(v, C.yyy) );
  vec3 x0 =   v - i + dot(i, C.xxx) ;

// Other corners
  vec3 g = step(x0.yzx, x0.xyz);
  vec3 l = 1.0 - g;
  vec3 i1 = min( g.xyz, l.zxy );
  vec3 i2 = max( g.xyz, l.zxy );

  //   x0 = x0 - 0.0 + 0.0 * C.xxx;
  //   x1 = x0 - i1  + 1.0 * C.xxx;
  //   x2 = x0 - i2  + 2.0 * C.xxx;
  //   x3 = x0 - 1.0 + 3.0 * C.xxx;
  vec3 x1 = x0 - i1 + C.xxx;
  vec3 x2 = x0 - i2 + C.yyy; // 2.0*C.x = 1/3 = C.y
  vec3 x3 = x0 - D.yyy;      // -1.0+3.0*C.x = -0.5 = -D.y

// Permutations
  i = mod2893(i); 
  vec4 p = permute( permute( permute( 
             i.z + vec4(0.0, i1.z, i2.z, 1.0 ))
           + i.y + vec4(0.0, i1.y, i2.y, 1.0 )) 
           + i.x + vec4(0.0, i1.x, i2.x, 1.0 ));

// Gradients: 7x7 points over a square, mapped onto an octahedron.
// The ring size 17*17 = 289 is close to a multiple of 49 (49*6 = 294)
  float n_ = 0.142857142857; // 1.0/7.0
  vec3  ns = n_ * D.wyz - D.xzx;

  vec4 j = p - 49.0 * floor(p * ns.z * ns.z);  //  mod(p,7*7)

  vec4 x_ = floor(j * ns.z);
  vec4 y_ = floor(j - 7.0 * x_ );    // mod(j,N)

  vec4 x = x_ *ns.x + ns.yyyy;
  vec4 y = y_ *ns.x + ns.yyyy;
  vec4 h = 1.0 - abs(x) - abs(y);

  vec4 b0 = vec4( x.xy, y.xy );
  vec4 b1 = vec4( x.zw, y.zw );

  //vec4 s0 = vec4(lessThan(b0,0.0))*2.0 - 1.0;
  //vec4 s1 = vec4(lessThan(b1,0.0))*2.0 - 1.0;
  vec4 s0 = floor(b0)*2.0 + 1.0;
  vec4 s1 = floor(b1)*2.0 + 1.0;
  vec4 sh = -step(h, vec4(0.0));

  vec4 a0 = b0.xzyw + s0.xzyw*sh.xxyy ;
  vec4 a1 = b1.xzyw + s1.xzyw*sh.zzww ;

  vec3 p0 = vec3(a0.xy,h.x);
  vec3 p1 = vec3(a0.zw,h.y);
  vec3 p2 = vec3(a1.xy,h.z);
  vec3 p3 = vec3(a1.zw,h.w);

//Normalise gradients
  vec4 norm = taylorInvSqrt(vec4(dot(p0,p0), dot(p1,p1), dot(p2, p2), dot(p3,p3)));
  p0 *= norm.x;
  p1 *= norm.y;
  p2 *= norm.z;
  p3 *= norm.w;

// Mix final noise value
  vec4 m = max(0.6 - vec4(dot(x0,x0), dot(x1,x1), dot(x2,x2), dot(x3,x3)), 0.0);
  m = m * m;
  return 42.0 * dot( m*m, vec4( dot(p0,x0), dot(p1,x1), 
                                dot(p2,x2), dot(p3,x3) ) );
  }


vec3 hsv2rgb(vec3 c){
    vec4 K = vec4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);
    vec3 p = abs(fract(c.xxx + K.xyz) * 6.0 - K.www);
    return c.z * mix(K.xxx, clamp(p - K.xxx, 0.0, 1.0), c.y);
}

float getNoise(vec2 uv, float t){
    
    float TRAVEL_SPEED = _speed;
    
    //octave 1
    float SCALE = _scale;
    float noise = snoise( vec3(uv.x*SCALE ,uv.y*SCALE * 4. + t*TRAVEL_SPEED , 0.));
    
    //octave 2 - more detail
    SCALE = 6.0;
    noise += snoise( vec3(uv.x*SCALE + t*TRAVEL_SPEED,uv.y*SCALE , 0))* 0.2 ;
    
    //move noise into 0 - 1 range    
    noise = (noise/2. + 0.5);
    
    return noise;
    
}

float getDepth(float n, float cutoff, float steps){
 
    //given a 0-1 value return a depth,
    
    //remap remaining non-cutoff region to 0 - 1
	float d = (n - cutoff) / (1. - cutoff); 
        
    //step it
    d = floor(d*steps)/steps;
    
    return d;
    
}

float hash(vec2 p){
	vec3 p3  = fract(vec3(p.xyx) * .1031);
	p3 += dot(p3, p3.yzx + 33.33);
	return fract((p3.x + p3.y) * p3.z);
}
	
float luminance(vec3 color) {
	const vec3 magic = vec3(0.2125, 0.7154, 0.0721);
	return dot(magic, color);
}

void fragment() {
	float steps = 4.;
	float cutoff = 0.15; //depth less than this, show black

	vec2 uv = UV;
    uv.x *= 5.0;
    float t = TIME * 3.0;    
    vec3 col = vec3(0.0);
    
   	float noise = getNoise(uv, t);
    
    //shape cutoff to get higher further up the screen
    cutoff = 1.1 - uv.y * 2.5;    
       
    if (noise < cutoff){  
        col = vec3(0.);
    }else{
		//fire
        float d = pow(getDepth(noise, cutoff, steps),0.7);        
        vec3 hsv = vec3(d *0.17,0.8 - d/4., d + 0.8);
        col = hsv2rgb(hsv);
	}  
	
	ALPHA = col.r;
	
	ALBEDO = vec3(col) * 1.5;
	
}
