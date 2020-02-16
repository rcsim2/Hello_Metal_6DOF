//
//  Shaders.metal
//  hello_metal_6dof
//
//  Created by Rik Goossens on 11/02/2020.
//  Copyright © 2020 cgdemy.com. All rights reserved.
//

// File for Metal kernel and shader functions

#include <metal_stdlib>
#include <simd/simd.h>

// Including header shared between this Metal shader code and Swift/C code executing Metal API commands
#import "ShaderTypes.h"

using namespace metal;

typedef struct
{
    float3 position [[attribute(VertexAttributePosition)]];
    float2 texCoord [[attribute(VertexAttributeTexcoord)]];
} Vertex;

typedef struct
{
    float4 position [[position]];
    float2 texCoord;
} ColorInOut;


// Cube
vertex ColorInOut vertexShader(Vertex in [[stage_in]],
                               constant Uniforms & uniforms [[ buffer(BufferIndexUniforms) ]])
{
    ColorInOut out;

    float4 position = float4(in.position, 1.0);
    out.position = uniforms.projectionMatrix * uniforms.modelViewMatrix * position;
    out.texCoord = in.texCoord;

    return out;
}


// Quad
vertex ColorInOut vertexShader2(Vertex in [[stage_in]],
                               constant Uniforms & uniforms [[ buffer(BufferIndexUniforms) ]])
{
    ColorInOut out;

    float4 position = float4(in.position, 2.0);
    //out.position = uniforms.projectionMatrix2 * uniforms.modelViewMatrix2 * position;
    
    // We want the quad to be just bottom left in screen coordinates
    // Looks OK like this but it is still scaling with screen resize
    // Or do we want that?
    position.x -= 1.0;
    position.y -= 1.8;
    //position.z += 0.0; // does not do much but must always stay in z-clip planes???
    
    out.position = position;
    
    
    out.texCoord = in.texCoord;
    
    // For our font rendering stuff we cannot properly access the texture coordinates here.
    //out.texCoord = in.texCoord - 0.1;

    return out;
}







fragment float4 fragmentShader(ColorInOut in [[stage_in]],
                               constant Uniforms & uniforms [[ buffer(BufferIndexUniforms) ]],
                               texture2d<half> colorMap     [[ texture(TextureIndexColor) ]])
{
    constexpr sampler colorSampler(mip_filter::linear,
                                   mag_filter::linear,
                                   min_filter::linear);

    half4 colorSample   = colorMap.sample(colorSampler, in.texCoord.xy);

    return float4(colorSample);
}
