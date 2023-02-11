float weight_z_exp(float center_z, float offset_z, int x, int y, int step_width,
               sampler2D depth_texture, vec2 text_coord,
               vec2 pixel_step, float fwidth_h, float z_multiplier, float nabla_max)
{
    float h = fwidth_h;
    float offset_z_x =
        (texture(depth_texture,   text_coord+vec2(pixel_step.x*h, 0)).x
         - texture(depth_texture, text_coord+vec2(0,              0)).x)

        / (h*pixel_step.x);

    float offset_z_y =
        (texture(depth_texture,   text_coord+vec2(0, pixel_step.y*h)).x
         - texture(depth_texture, text_coord+vec2(0,              0)).x)

        / (h*pixel_step.y);

    vec2 nabla = vec2(offset_z_x, offset_z_y);
    float f_width = abs(offset_z_x) + abs(offset_z_y);

    // if (length(nabla) > nabla_max) {
        // return -abs(offset_z - center_z) * z_multiplier;
    // } else {
        // NOTE(Felix): paper
        return -abs(offset_z - center_z) * z_multiplier /
            max(1e-8, abs(dot(nabla, vec2(x, y))) * step_width);

    // }

    // NOTE(Felix): falcor
    // return
    //     -(abs(offset_z - center_z) * z_multiplier)
    //     /
    //     (max(f_width, 1e-8) * step_width * length(vec2(x,y)));

    // NOTE(Felix): empirical
    // return
        // -abs(offset_z - center_z) * z_multiplier
        // /
        // (5e-3 * step_width * length(vec2(x, y)));

}

float weight_l_exp() {
    return 0;
}

float weight_n(vec3 center_normal, vec3 offset_normal) {
    float sigma_n = 128;

    return pow(max(0, (dot(center_normal, offset_normal))), sigma_n);
}