#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in float aColor;

out vec3 FragPos;
out vec3 Normal;
out vec3 objectColor;

uniform mat4 view;
uniform mat4 projection;

void main()
{
    // FragPos = vec3(model * vec4(aPos, 1.0));
    FragPos = aPos;
    Normal = aNormal;  
    int color_int = int(aColor);  // I Don't know why here need a cast
    objectColor = vec3( ( ( color_int >> 16 ) & 0xff ), 
                        ( ( color_int >> 8 ) & 0xff  ),
                        ( ( color_int >> 0 ) & 0xff ) ) / 255.0; 
    gl_Position = projection * view * vec4(FragPos, 1.0);
}