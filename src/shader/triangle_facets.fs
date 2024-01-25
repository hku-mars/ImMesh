#version 330 core
out vec4 FragColor;

in vec3 Normal;  
in vec3 FragPos;  
in vec3 objectColor;

uniform bool if_light;
uniform vec3 lightPos; 
uniform vec3 lightColor;

void main()
{
    // ambient
    float ambientStrength = 0.2;
    vec3 ambient = ambientStrength * lightColor;
  	
    // diffuse
    if ( if_light )
    {
        vec3 norm = normalize( Normal );
        vec3 lightDir = normalize( lightPos - FragPos );
        // float diff = max((dot(norm, lightDir)), 0.0);
        float diff = max( abs( dot( norm, lightDir ) ) * 0.5, 0.0 ); // Set abs to enable two-sided lighting
        vec3  diffuse = diff * lightColor;

        vec3 result = ( ambient + diffuse ) * objectColor;
        FragColor = vec4( result, 1.0 );
    }
    else
    {
        FragColor = vec4( objectColor, 1.0 );
    }
    // FragColor = vec4(vec3(1,1,1), 1.0);
} 