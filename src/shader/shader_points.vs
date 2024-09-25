#version 330 core
 
layout (location = 0) in vec3 aPos;
layout (location = 1) in float aIntensity;
 
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform bool isIntensity;
uniform float minIntensity;
uniform float maxIntensity;
uniform float minDistance;
uniform float maxDistance;

out vec3 pointColor;

//函数声明
vec3 fakeColor(float value);


void main()
{
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    vec3 fColor = vec3(0.5f, 1.0f, 1.0f);
    if(isIntensity){ // 按强度着色
        float intensity_normalized = (aIntensity - minIntensity) / (maxIntensity - minIntensity);
        fColor = fakeColor(intensity_normalized);  // 灰度颜色
    }
    else{ // 按距离着色
        float distance = pow(aPos.x, 2) + pow(aPos.y, 2) + pow(aPos.z, 2);
        float distance_normalized = (distance - minDistance) / (maxDistance - minDistance); 
        fColor = fakeColor(distance_normalized);  // 灰度颜色
    }

    pointColor =  fColor;
}


vec3 fakeColor(float value) {
    // Clamp value between 0 and 1
    value = clamp(value, 0.0, 1.0);
    
    vec3 color;
    // Define the six soft color stops: Orange, Yellow, Green, Cyan, Blue, Indigo
    if (value < 1.0 / 6.0) {
        // Orange to Yellow (value: 0 to 1/6)
        float t = value * 6.0;
        color = mix(vec3(1.0, 0.7, 0.4), vec3(1.0, 1.0, 0.6), t);  // Soft orange to soft yellow
    } else if (value < 2.0 / 6.0) {
        // Yellow to Green (value: 1/6 to 2/6)
        float t = (value - 1.0 / 6.0) * 6.0;
        color = mix(vec3(1.0, 1.0, 0.6), vec3(0.6, 1.0, 0.6), t);  // Soft yellow to soft green
    } else if (value < 3.0 / 6.0) {
        // Green to Cyan (value: 2/6 to 3/6)
        float t = (value - 2.0 / 6.0) * 6.0;
        color = mix(vec3(0.6, 1.0, 0.6), vec3(0.6, 1.0, 1.0), t);  // Soft green to soft cyan
    } else if (value < 4.0 / 6.0) {
        // Cyan to Blue (value: 3/6 to 4/6)
        float t = (value - 3.0 / 6.0) * 6.0;
        color = mix(vec3(0.6, 1.0, 1.0), vec3(0.6, 0.6, 1.0), t);  // Soft cyan to soft blue
    } else if (value < 5.0 / 6.0) {
        // Blue to Indigo (value: 4/6 to 5/6)
        float t = (value - 4.0 / 6.0) * 6.0;
        color = mix(vec3(0.6, 0.6, 1.0), vec3(0.5, 0.4, 0.8), t);  // Soft blue to soft indigo
    } else {
        // Indigo to Deep Indigo (value: 5/6 to 1)
        float t = (value - 5.0 / 6.0) * 6.0;
        color = mix(vec3(0.5, 0.4, 0.8), vec3(0.4, 0.3, 0.6), t);  // Indigo to deep indigo
    }

    return color;
}