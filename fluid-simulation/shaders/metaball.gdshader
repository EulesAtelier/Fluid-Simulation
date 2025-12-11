shader_type spatial;
render_mode unshaded, cull_back, depth_draw_opaque;

uniform int metaball_count = 0;

uniform vec3 metaball_pos[128];
uniform float metaball_radius;

uniform float iso_threshold = 1.0;
uniform float smoothness = 0.6;
uniform int max_steps = 128;
uniform float hit_epsilon = 0.01;

// Bounds of the volume (match your box mesh)
uniform vec3 box_min = vec3(-1.0, -1.0, -1.0);
uniform vec3 box_max = vec3( 1.0,  1.0,  1.0);

uniform vec4 fluid_color : source_color = vec4(0.2, 0.7, 1.0, 1.0);


float metaball_field(vec3 p) {
    float f = 0.0;
    for (int i = 0; i < metaball_count; i++) {
        float r = metaball_radius;
        float d = length(p - metaball_pos[i]);
        f += (r * r) / (d * d + smoothness);
    }
    return f;
}
float smooth_min(float a, float b, float k) {
    float h = clamp(0.5 + 0.5*(b - a)/k, 0.0, 1.0);
    return mix(b, a, h) - k*h*(1.0 - h);
}

float scene_sdf(vec3 p) {
    float field = 0.0;

    for (int i = 0; i < metaball_count; i++) {
        float r = metaball_radius;
        float d = length(p - metaball_pos[i]);

        float x = clamp(1.0 - d / r, 0.0, 1.0);

        field += x * x * x * (6.0 * x * x - 15.0 * x + 10.0);
    }

    return iso_threshold - field;
}

vec3 estimate_normal(vec3 p) {
    float e = 0.002;
    return normalize(vec3(
        scene_sdf(p + vec3(e,0,0)) - scene_sdf(p - vec3(e,0,0)),
        scene_sdf(p + vec3(0,e,0)) - scene_sdf(p - vec3(0,e,0)),
        scene_sdf(p + vec3(0,0,e)) - scene_sdf(p - vec3(0,0,e))
    ));
}

//If I update this I can get the simulation to render while  inside the mesh, but I like the current state so will keep it as is.
bool ray_box_intersect(vec3 ro, vec3 rd, out float t0, out float t1) {
    vec3 inv = 1.0 / rd;
    vec3 tmin = (box_min - ro) * inv;
    vec3 tmax = (box_max - ro) * inv;

    vec3 t1c = min(tmin, tmax);
    vec3 t2c = max(tmin, tmax);

    t0 = max(max(t1c.x, t1c.y), t1c.z);
    t1 = min(min(t2c.x, t2c.y), t2c.z);

    return (t1 > max(t0, 0.0));
}

void fragment() {

    vec2 uv = SCREEN_UV * 2.0 - 1.0;

    vec4 view = INV_PROJECTION_MATRIX * vec4(uv, 1.0, 1.0);
    vec3 rd = normalize((INV_VIEW_MATRIX * vec4(view.xyz, 0.0)).xyz);
    vec3 ro = CAMERA_POSITION_WORLD;

    float t0, t1;
    if (!ray_box_intersect(ro, rd, t0, t1)) {
        discard;
    }
    t0 = max(t0, 0.0);

    float t = t0;
    bool hit = false;

    for (int i = 0; i < max_steps; i++) {
        vec3 p = ro + rd * t;
        float d = scene_sdf(p);

        if (d < hit_epsilon) {
            hit = true;
            break;
        }

        t += max(d, 0.01);
        if (t > t1) break;
    }

    if (!hit) {
        discard;
    }

    vec3 pos = ro + rd * t;
    vec3 n = estimate_normal(pos);

    vec3 light_dir = normalize(vec3(0.4, 1.2, 0.3));
    float diff = clamp(dot(n, light_dir), 0.2, 1.0);

    ALBEDO = fluid_color.rgb * diff;
    ALPHA = fluid_color.a;
}
