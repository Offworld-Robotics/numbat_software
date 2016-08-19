precision highp float;

varying vec2 uv;

uniform sampler2D tex;

uniform float frame_offset;  // Which 'direction' to point in
uniform float fov_degrees;

float radial_offset = 0.7;           // How far from center to start drawing
float radial_scale = 1.6;            // Widens radius slice that is seen
float aspect_x_v_y = 1.0;            // i.e 16x9 is 16/9 - 1 is square

// Where the 360 camera center is in the frame
vec2 center;
center.x = 0.5;
center.y = 0.5;

void main(void) {
	float theta_scale = 360/fov_degrees;
	float this_theta = uv.x*3.14*2.0/theta_scale + frame_offset;
	float this_r = (1.0/radial_scale)*(1.0 + radial_offset - uv.y)/2.0;
	vec2 uv_out = uv;
	uv_out.x = (cos(this_theta) * this_r + center.x)/aspect_x_v_y;
	uv_out.y = sin(this_theta) * this_r + center.y;
	gl_FragColor = texture2D(tex,uv_out);
}

