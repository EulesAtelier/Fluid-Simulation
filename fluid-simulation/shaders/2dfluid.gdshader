shader_type canvas_item;

uniform int particle_count;
uniform vec2 particle_positions[1024];
uniform vec2 rect_size;
uniform float radius = 32.0;
uniform float threshold = 1.0;

uniform vec4 color : source_color = vec4(1.0, 1.0, 1.0, 1.0); // RGB + alpha
uniform float opacity = 1.0;

void fragment() {
    vec2 uv = UV * rect_size;
    float field = 0.0;

    for (int i = 0; i < particle_count; i++) {
        float dist = distance(uv, particle_positions[i]);
        field += radius * radius / (dist * dist + 0.0001);
    }

    float t = smoothstep(threshold - 0.1, threshold, field);

    if (t > 0.0) {
        COLOR = vec4(color.rgb, color.a * opacity * t);
    } else {
        discard;
    }
}
