#version 450
#extension GL_EXT_debug_printf : enable

layout(location = 0) in vec3 fragColor;
layout(location = 1) in vec2 fragTexCoord;

layout(binding = 1) uniform sampler2D texSampler;
layout(set = 0, binding = 2) uniform MousePositionBufferObject {
    vec4 mousePosition;
    vec4 playerPosition;
} ubo;
layout(set = 0, binding = 3) readonly buffer SSBO {
    int map[];
};

struct RigidBodySquareObject{
    vec4 centerOfMass;
    vec4 rotation;
    vec4 acceleration;
    vec4 velocity;
    int mass;
    int scale;
};
layout(set = 0, binding = 4) readonly buffer RigidBodySSBO {
    RigidBodySquareObject rigidBodies[];
};

layout(location = 0) out vec4 outColor;




#define pi 3.1415926535897932384626433832795

vec3 polarToCartesian(float r, vec2 angles) {
    float x = r * sin(angles.y) * cos(angles.x);
    float y = r * cos(angles.y);
    float z = r * sin(angles.x) * sin(angles.y);
    return vec3(x,y,z);
}

vec3 rotateY(vec2 lookAngle, vec3 ray) {
    float zalign = pi / 2.0 + lookAngle.x;
    mat4 alignz = mat4(
        cos(zalign), 0, sin(zalign), 0, // col 1
        0, 1, 0, 0, // col 2
        -sin(zalign), 0, cos(zalign), 0, // col 3
        0, 0, 0, 1 // col 4
    );
    
    mat4 inverseAlignz = mat4(
        cos(zalign), 0, -sin(zalign), 0, // col 1
        0, 1, 0, 0, // col 2
        sin(zalign), 0, cos(zalign), 0, // col 3
        0, 0, 0, 1 // col 4
    );
    
    // look up is negative because lookangle is opposite of rotation angle
    mat4 rotateUp = mat4(
        cos(-lookAngle.y), sin(-lookAngle.y), 0, 0,
        -sin(-lookAngle.y), cos(-lookAngle.y), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    );
    
    vec4 ray4 = vec4(ray, 1);
    
    return (inverseAlignz * rotateUp * alignz * ray4).xyz;

}

float hash13(vec3 p3)
{
	p3  = fract(p3 * .1031);
    p3 += dot(p3, p3.zyx + 31.32);
    return fract((p3.x + p3.y) * p3.z);
}

vec3 hash33(vec3 p3)
{
	p3 = fract(p3 * .5);
    p3 += dot(p3, p3.yxz+19.19);
    return fract((p3.xxy + p3.yxx)*p3.zyx);
}

vec3 pointOctreeNoise(vec3 pos, float seed) {
    float hash = hash13(pos);
    int maxSize = 8;
    int isSolid = 1;
    int x = int(pos.x) % 256;
    int y = int(pos.y) % 256;
    int z = int(pos.z) % 256;
    
    for (int i = 1; i <= 8; i++) {
        if (x % (1 << i) > 0 || y % (1 << i) > 0 || z % (1 << i) > 0) {
            maxSize = i - 1;
            break;
        }
    }
    if (hash < .9) {
        isSolid = 0;
    }
    
    float sizeHash = hash13(vec3(hash, maxSize, hash13(vec3(pos.y, pos.z, pos.x))));
    // cant be 256 if floor
    int size = int(floor(sizeHash * float(maxSize + 1)));
    
    return vec3(1 << size, isSolid, hash);
}

vec3 octreeNoise(vec3 pos, float seed) {
    float hash = hash13(pos);
    int maxSize = 8;
    int isSolid = 1;
    int x = int(pos.x) % 256;
    int y = int(pos.y) % 256;
    int z = int(pos.z) % 256;
    
    for (int i = 8; i >= 0; i--) {
        if (i != 8 && (x >> i) % 2 == 0 && (y >> i) % 2 == 0 && (z >> i) % 2 == 0) {
            continue;
        }
        
        vec3 parentPos = vec3(
            (int(pos.x) >> 8 << 8) + (x >> i << i), 
            (int(pos.y) >> 8 << 8) + (y >> i << i), 
            (int(pos.z) >> 8 << 8) + (z >> i << i)
        );
        vec3 parent = pointOctreeNoise(
            parentPos,
            seed
        );
        
        vec3 parentBounds = parentPos + parent.x;
        
        if (pos.x < parentBounds.x && pos.y < parentBounds.y && pos.z < parentBounds.z) {
            return parent;
        }
    }
}


vec3 rotate(vec4 rotation, vec4 ray) {
    mat4 xRotationMatrix = mat4(1, 0, 0, 0,
        0, cos(rotation.x), -sin(rotation.x), 0,
        0, sin(rotation.x), cos(rotation.x), 0,
        0, 0, 0, 1);
    mat4 yRotationMatrix = mat4(cos(rotation.y), 0, sin(rotation.y), 0,
        0, 1, 0, 0,
        -sin(rotation.y), 0, cos(rotation.y), 0,
        0, 0, 0, 1);
    mat4 zRotationMatrix = mat4(cos(rotation.z), sin(rotation.z), 0, 0,
        -sin(rotation.z), cos(rotation.z), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1);

    return (zRotationMatrix * yRotationMatrix * xRotationMatrix * ray).xyz;
}

// same rigidBody shown behind player (ray casted both ways)
vec3 rigidBodyRayTransformation(RigidBodySquareObject rigidBody, vec3 rayEnd) {
    // inverse translation assuming scale of 1
    vec4 playerPosition = vec4(ubo.playerPosition.x, ubo.playerPosition.y, ubo.playerPosition.z, 1);
    vec4 translatedRayStart = playerPosition - rigidBody.centerOfMass;
    vec4 translatedRayEnd = playerPosition + vec4(rayEnd, 0) - rigidBody.centerOfMass;

    // inverse rotation
    vec3 rotatedRayStart = rotate(-rigidBody.rotation, translatedRayStart);
    vec3 rotatedRayEnd = rotate(-rigidBody.rotation, translatedRayEnd);

    // detect collision
    float xZeroIntersectionSteps = (-.5f - rotatedRayStart.x) / (rotatedRayEnd.x - rotatedRayStart.x);
    float xOneIntersectionSteps = (.5f - rotatedRayStart.x) / (rotatedRayEnd.x - rotatedRayStart.x);
    float yZeroIntersectionSteps = (-.5f - rotatedRayStart.y) / (rotatedRayEnd.y - rotatedRayStart.y);
    float yOneIntersectionSteps = (.5f - rotatedRayStart.y) / (rotatedRayEnd.y - rotatedRayStart.y);
    float zZeroIntersectionSteps = (-.5f - rotatedRayStart.z) / (rotatedRayEnd.z - rotatedRayStart.z);
    float zOneIntersectionSteps = (.5f - rotatedRayStart.z) / (rotatedRayEnd.z - rotatedRayStart.z);

    vec3 rayStep = rotatedRayEnd - rotatedRayStart;

    vec3 xZeroIntersection = xZeroIntersectionSteps * rayStep + rotatedRayStart;
    vec3 xOneIntersection = xOneIntersectionSteps * rayStep + rotatedRayStart;
    vec3 yZeroIntersection = yZeroIntersectionSteps * rayStep + rotatedRayStart;
    vec3 yOneIntersection = yOneIntersectionSteps * rayStep + rotatedRayStart;
    vec3 zZeroIntersection = zZeroIntersectionSteps * rayStep + rotatedRayStart;
    vec3 zOneIntersection = zOneIntersectionSteps * rayStep + rotatedRayStart;

    if (rotatedRayStart.x < -.5) { 
        if (xZeroIntersection.y <= .5 && xZeroIntersection.y >= -.5 && xZeroIntersection.z <= .5 && xZeroIntersection.z >= -.5) {
            return vec3(1, 0, 0);
        }
    }
    if (rotatedRayStart.x > .5) {
        if (xOneIntersection.y <= .5 && xOneIntersection.y >= -.5 && xOneIntersection.z <= .5 && xOneIntersection.z >= -.5) {
            return vec3(1, 0, 0);
        }
    }
    if (rotatedRayStart.y < -.5) { 
        if (yZeroIntersection.x <= .5 && yZeroIntersection.x >= -.5 && yZeroIntersection.z <= .5 && yZeroIntersection.z >= -.5) {
            return vec3(0, 1, 0);
        }
    }
    if (rotatedRayStart.y > .5) { 
        if (yOneIntersection.x <= .5 && yOneIntersection.x >= -.5 && yOneIntersection.z <= .5 && yOneIntersection.z >= -.5) {
            return vec3(0, 1, 0);
        }
    }
    if (rotatedRayStart.z < -.5) { 
        if (zZeroIntersection.x <= .5 && zZeroIntersection.x >= -.5 && zZeroIntersection.y <= .5 && zZeroIntersection.y >= -.5) {
            return vec3(0, 0, 1);
        }
    }
    if (rotatedRayStart.z > .5) { 
        if (zOneIntersection.x <= .5 && zOneIntersection.x >= -.5 && zOneIntersection.y <= .5 && zOneIntersection.y >= -.5) {
            return vec3(0, 0, 1);
        }
    }

    return vec3(0, 0, 0);
}




void main() {
    //outColor = texture(texSampler, fragTexCoord);
    // resolution is 1920x1080
    //outColor = vec4(gl_FragCoord.x/1920, gl_FragCoord.y/1080, 0, 1.0);

    //debugPrintfEXT("FRAG: %.2f %.2f | ", gl_FragCoord.x, gl_FragCoord.y);
    //debugPrintfEXT("mousePositiion: %.2f %.2f %.2f %.2f \n ", ubo.mousePosition.x, ubo.mousePosition.y, ubo.mousePosition.z, ubo.mousePosition.w);

    // y starts from top x starts from left
    vec2 uv = (gl_FragCoord.xy * vec2(1, -1) + vec2(0, ubo.mousePosition.w))/ubo.mousePosition.zw - vec2(.5, .5);
    vec2 muv = ubo.mousePosition.xy/ubo.mousePosition.zw - vec2(.5, .5);
    
    float ytox = ubo.mousePosition.z / ubo.mousePosition.w;
    uv.x *= ytox;
    uv *= 2.0;
 
    vec2 lookAngle = vec2(2.0 * pi * muv.x, 2.0 * pi * muv.y);
    float r = sqrt(1.0 + uv.x * uv.x + uv.y * uv.y);
    float rx = sqrt(1.0 + uv.x * uv.x);
    vec2 polarAngle = vec2(asin(1.0/rx), asin(rx/r));
    if (uv.x < 0.0) {
        polarAngle.x = pi - polarAngle.x;
    }
    if (uv.y < 0.0) {
        polarAngle.y = pi - polarAngle.y;
    }

    // lookAngle is opposite of polarAngle (looking right is positive lookAngle which respectively should subtract from xRotated axis because the look axis and player axis are opposite)
    // same for looking up
    vec2 xRotated = vec2(polarAngle.x - lookAngle.x, polarAngle.y);
    vec3 xRotatedRay = polarToCartesian(r, xRotated);
    vec3 ray = rotateY(lookAngle, xRotatedRay);
    
    int dim = 1000;
    vec3 col = vec3(1, 1, 1);
    float ix = ubo.playerPosition.x;
    float iy = ubo.playerPosition.y;
    float iz = ubo.playerPosition.z;
    float stepSize = 1.0;
        
    // rigidBody ray march
    // shortest dist to COM optimization 
    for (int i = 0; i < 2; i++) {
        vec3 color = rigidBodyRayTransformation(rigidBodies[i], ray);
        if (color != vec3(0, 0, 0)) {
            outColor = vec4(color, 1.0f);
            return;
        }
    }

    // set ray march dist
    for (int i = 0; i < 200; i++){
        //float diffx = ray.x > 0.0 ? 1.0 + floor(ix) - ix : 1.0 - ceil(ix) + ix;
        //float diffy = ray.y > 0.0 ? 1.0 + floor(iy) - iy : 1.0 - ceil(iy) + iy;
        //float diffz = ray.z > 0.0 ? 1.0 + floor(iz) - iz : 1.0 - ceil(iz) + iz;
        
        float diffx = ray.x > 0.0 ? stepSize - float(int(floor(ix)) % int(stepSize)) - fract(ix) : float(int(floor(ix)) % int(stepSize)) + fract(ix);
        float diffy = ray.y > 0.0 ? stepSize - float(int(floor(iy)) % int(stepSize)) - fract(iy) : float(int(floor(iy)) % int(stepSize)) + fract(iy);
        float diffz = ray.z > 0.0 ? stepSize - float(int(floor(iz)) % int(stepSize)) - fract(iz) : float(int(floor(iz)) % int(stepSize)) + fract(iz);
       
        diffx = diffx > 0.0 ? diffx : stepSize;
        diffy = diffy > 0.0 ? diffy : stepSize;
        diffz = diffz > 0.0 ? diffz : stepSize;

        float wx = abs(diffx / ray.x);
        float wy = abs(diffy / ray.y);
        float wz = abs(diffz / ray.z);
        
        float minw = wx;
        if (wy < minw) {
            minw = wy;
        }
        if (wz < minw) {
            minw = wz;
        }
        
        ix += ray.x * minw;
        iy += ray.y * minw;
        iz += ray.z * minw;
        if (abs(ix - round(ix)) < .01) {
            ix = round(ix);
        }
        if (abs(iy - round(iy)) < .01) {
            iy = round(iy);
        }
        if (abs(iz - round(iz)) < .01) {
            iz = round(iz);
        }
        
        // causing rays to go through border of map at voxel borders
        if (ix < 0 || ix >= float(dim)) {
            break;
        }
        if (iy < 0 || iy >= float(dim)) {
            col = vec3(0, 0, 1);
            break;
        }
        if (iz < 0 || iz >= float(dim)) {
            break;
        }

        int ycoord = int(floor(iy));
        int zcoord = int(floor(iz));
        int xcoord = int(floor(ix));
        
        if (ix == round(ix) && ray.x < 0.0) {
            xcoord--;
        }
        if (iy == round(iy) && ray.y < 0.0) {
            ycoord--;
        }
        if (iz == round(iz) && ray.z < 0.0) {
            zcoord--;
        }
        else if (map[xcoord * 1000000 + ycoord * 1000 + zcoord] > 0) {
            // borders
            if (fract(ix) < .01 && fract(iy) < .01 || 
                fract(iy) < .01 && fract(iz) < .01 || 
                fract(ix) < .01 && fract(iz) < .01) {
                col = vec3(0, 0, 0);
                break;
            }
            col = vec3(1, 0, 0);
            break;
        }
        /*
        vec3 n = octreeNoise(vec3(xcoord, ycoord, zcoord), 1.0);
        if (n.y < 1.0) {
            stepSize = n.x;
        }
        else {
            col = hash33(vec3(n.z, n.z * 2.0, n.z * 3.0));
            col += vec3(
                int(ix) % int(n.x) == 0 && fract(ix) == 0.0 ? .1 : 0.0, 
                int(iy) % int(n.x) == 0 && fract(iy) == 0.0 ? .1 : 0.0, 
                int(iz) % int(n.x) == 0 && fract(iz) == 0.0 ? .1 : 0.0
            );
            break;
        }
        */
    }

    // Output to screen
    outColor = vec4(col,1.0);
}