// Git test13
//
//  Renderer.m
//  hello_metal_6dof
//
//  Created by Rik Goossens on 11/02/2020.
//  Copyright © 2020 cgdemy.com. All rights reserved.
//


#import <simd/simd.h>
#import <ModelIO/ModelIO.h>

#import "Renderer.h"

// Include header shared between C code here, which executes Metal API commands, and .metal files
#import "ShaderTypes.h"

#import "AAPLMathUtilities.h"


static const NSUInteger kMaxBuffersInFlight = 3;

static const size_t kAlignedUniformsSize = (sizeof(Uniforms) & ~0xFF) + 0x100;



@implementation Renderer
{
    dispatch_semaphore_t _inFlightSemaphore;
    id <MTLDevice> _device;
    id <MTLCommandQueue> _commandQueue;

    id <MTLBuffer> _dynamicUniformBuffer;
    id <MTLRenderPipelineState> _pipelineState;
    id <MTLDepthStencilState> _depthState;
    id <MTLTexture> _colorMap;
    MTLVertexDescriptor *_mtlVertexDescriptor;
    
    // Quad
    id <MTLTexture> _fontTexture;
    //MTLVertexDescriptor *_mtlVertexDescriptor2;
    id <MTLRenderPipelineState> _pipelineState2;



    

    uint32_t _uniformBufferOffset;

    uint8_t _uniformBufferIndex;

    void* _uniformBufferAddress;

    matrix_float4x4 _projectionMatrix;
    
    // Quad
    matrix_float4x4 _projectionMatrix2;


    float _rotation;
    float _rotation2;


    MTKMesh *_mesh;
    
    // Quad
    MTKMesh *_mesh2;
    
    
    matrix_float4x4 modelMatrix;
    matrix_float4x4 viewMatrix;
    
    
    vector_float3 vRight;
    vector_float3 vUp;
    vector_float3 vForward;
    
    
}



-(nonnull instancetype)initWithMetalKitView:(nonnull MTKView *)view;
{
    self = [super init];
    if(self)
    {
        _device = view.device;
        _inFlightSemaphore = dispatch_semaphore_create(kMaxBuffersInFlight);
        [self _loadMetalWithView:view];
        [self _loadAssets];
    }

    return self;
}



- (void)_loadMetalWithView:(nonnull MTKView *)view;
{
    /// Load Metal state objects and initalize renderer dependent view properties
    
    // Cam
    _camX = 0.0;
    _camY = 0.0;
    _camZ = -8.0;
    
    _upkey_down = false;
    _downkey_down = false;
    _leftkey_down = false;
    _rightkey_down = false;
    
    // Init once
    modelMatrix = viewMatrix = matrix4x4_translation(0.0, 0.0, 0.0);
    viewMatrix = matrix4x4_translation(0.0, 0.0, -8.0);
    
    vRight   = vector3( 1.0f, 0.0f, 0.0f );
    vUp      = vector3( 0.0f, 1.0f, 0.0f );
    vForward = vector3( 0.0f, 0.0f, 1.0f );
    
    
    view.depthStencilPixelFormat = MTLPixelFormatDepth32Float_Stencil8;
    view.colorPixelFormat = MTLPixelFormatBGRA8Unorm_sRGB;
    view.sampleCount = 1;
    

    // Vertex
    _mtlVertexDescriptor = [[MTLVertexDescriptor alloc] init];

    // x,y,z
    _mtlVertexDescriptor.attributes[VertexAttributePosition].format = MTLVertexFormatFloat3;
    _mtlVertexDescriptor.attributes[VertexAttributePosition].offset = 0;
    _mtlVertexDescriptor.attributes[VertexAttributePosition].bufferIndex = BufferIndexMeshPositions;

    // u,v (or s,t)
    _mtlVertexDescriptor.attributes[VertexAttributeTexcoord].format = MTLVertexFormatFloat2;
    _mtlVertexDescriptor.attributes[VertexAttributeTexcoord].offset = 0;
    _mtlVertexDescriptor.attributes[VertexAttributeTexcoord].bufferIndex = BufferIndexMeshGenerics;

    _mtlVertexDescriptor.layouts[BufferIndexMeshPositions].stride = 12; // x,y,x 3*4=12 bytes
    _mtlVertexDescriptor.layouts[BufferIndexMeshPositions].stepRate = 1;
    _mtlVertexDescriptor.layouts[BufferIndexMeshPositions].stepFunction = MTLVertexStepFunctionPerVertex;

    _mtlVertexDescriptor.layouts[BufferIndexMeshGenerics].stride = 8; // u,v 2*4=8 bytes
    _mtlVertexDescriptor.layouts[BufferIndexMeshGenerics].stepRate = 1;
    _mtlVertexDescriptor.layouts[BufferIndexMeshGenerics].stepFunction = MTLVertexStepFunctionPerVertex;
    

    // Shaders
    id <MTLLibrary> defaultLibrary = [_device newDefaultLibrary];

    // Vertex shader (one for each model)
    id <MTLFunction> vertexFunction = [defaultLibrary newFunctionWithName:@"vertexShader"];
    id <MTLFunction> vertexFunction2 = [defaultLibrary newFunctionWithName:@"vertexShader2"];

    // Pixel shader
    id <MTLFunction> fragmentFunction = [defaultLibrary newFunctionWithName:@"fragmentShader"];

    
    // Pipeline Cube
    MTLRenderPipelineDescriptor *pipelineStateDescriptor = [[MTLRenderPipelineDescriptor alloc] init];
    pipelineStateDescriptor.label = @"MyPipeline";
    pipelineStateDescriptor.sampleCount = view.sampleCount;
    pipelineStateDescriptor.vertexFunction = vertexFunction;
    pipelineStateDescriptor.fragmentFunction = fragmentFunction;
    pipelineStateDescriptor.vertexDescriptor = _mtlVertexDescriptor;
    pipelineStateDescriptor.colorAttachments[0].pixelFormat = view.colorPixelFormat;
    pipelineStateDescriptor.depthAttachmentPixelFormat = view.depthStencilPixelFormat;
    pipelineStateDescriptor.stencilAttachmentPixelFormat = view.depthStencilPixelFormat;

    NSError *error = NULL;
    _pipelineState = [_device newRenderPipelineStateWithDescriptor:pipelineStateDescriptor error:&error];
    if (!_pipelineState)
    {
        NSLog(@"Failed to created pipeline state, error %@", error);
    }
    
    
    // Pipeline2 Quad
    MTLRenderPipelineDescriptor *pipelineStateDescriptor2 = [[MTLRenderPipelineDescriptor alloc] init];
    pipelineStateDescriptor2.label = @"MyPipeline2";
    pipelineStateDescriptor2.sampleCount = view.sampleCount;
    pipelineStateDescriptor2.vertexFunction = vertexFunction2; // this uses modelViewMatrix2
    pipelineStateDescriptor2.fragmentFunction = fragmentFunction;
    pipelineStateDescriptor2.vertexDescriptor = _mtlVertexDescriptor;
    pipelineStateDescriptor2.colorAttachments[0].pixelFormat = view.colorPixelFormat;
    pipelineStateDescriptor2.depthAttachmentPixelFormat = view.depthStencilPixelFormat;
    pipelineStateDescriptor2.stencilAttachmentPixelFormat = view.depthStencilPixelFormat;

    NSError *error2 = NULL;
    _pipelineState2 = [_device newRenderPipelineStateWithDescriptor:pipelineStateDescriptor2 error:&error2];
    if (!_pipelineState)
    {
        NSLog(@"Failed to created pipeline state, error %@", error2);
    }

    
    // Other stuff
    MTLDepthStencilDescriptor *depthStateDesc = [[MTLDepthStencilDescriptor alloc] init];
    depthStateDesc.depthCompareFunction = MTLCompareFunctionLess;
    depthStateDesc.depthWriteEnabled = YES;
    _depthState = [_device newDepthStencilStateWithDescriptor:depthStateDesc];

    NSUInteger uniformBufferSize = kAlignedUniformsSize * kMaxBuffersInFlight;

    _dynamicUniformBuffer = [_device newBufferWithLength:uniformBufferSize
                                                 options:MTLResourceStorageModeShared];

    _dynamicUniformBuffer.label = @"UniformBuffer";
    
    
    // Command
    _commandQueue = [_device newCommandQueue];
}



- (void)_loadAssets
{
    /// Load assets into metal objects

    NSError *error;

    MTKMeshBufferAllocator *metalAllocator = [[MTKMeshBufferAllocator alloc]
                                              initWithDevice: _device];
    

    // The Cube
    MDLMesh *mdlMesh = [MDLMesh newBoxWithDimensions:(vector_float3){4, 3, 6}
                                            segments:(vector_uint3){1, 1, 1}
                                        geometryType:MDLGeometryTypeTriangles
                                       inwardNormals:NO
                                           allocator:metalAllocator];

    MDLVertexDescriptor *mdlVertexDescriptor =
    MTKModelIOVertexDescriptorFromMetal(_mtlVertexDescriptor);

    mdlVertexDescriptor.attributes[VertexAttributePosition].name  = MDLVertexAttributePosition;
    mdlVertexDescriptor.attributes[VertexAttributeTexcoord].name  = MDLVertexAttributeTextureCoordinate;

    mdlMesh.vertexDescriptor = mdlVertexDescriptor;

    _mesh = [[MTKMesh alloc] initWithMesh:mdlMesh
                                   device:_device
                                    error:&error];

    if(!_mesh || error)
    {
        NSLog(@"Error creating MetalKit mesh %@", error.localizedDescription);
    }

    
    // The Cube's texture
    MTKTextureLoader* textureLoader = [[MTKTextureLoader alloc] initWithDevice:_device];

    NSDictionary *textureLoaderOptions =
    @{
      MTKTextureLoaderOptionTextureUsage       : @(MTLTextureUsageShaderRead),
      MTKTextureLoaderOptionTextureStorageMode : @(MTLStorageModePrivate)
      };

    _colorMap = [textureLoader newTextureWithName:@"Heli2"//@"Mir2"//@"ColorMap"
                                      scaleFactor:1.0
                                           bundle:nil
                                          options:textureLoaderOptions
                                            error:&error];

    if(!_colorMap || error)
    {
        NSLog(@"Error creating texture %@", error.localizedDescription);
    }

    
    
 
    // The Quad
//    MDLMesh *mdlMesh2 = [MDLMesh newPlaneWithDimensions:(vector_float2){4, 4}
//                                            segments:(vector_uint2){1, 1}
//                                        geometryType:MDLGeometryTypeTriangles
//                                           allocator:metalAllocator];
    // Let's make it a box first so we can easily see it
    MDLMesh *mdlMesh2 = [MDLMesh newBoxWithDimensions:(vector_float3){2, 1, 1}
                                             segments:(vector_uint3){1, 1, 1}
                                         geometryType:MDLGeometryTypeTriangles
                                        inwardNormals:NO
                                            allocator:metalAllocator];
    
      // We already have the vertex descriptor
//    MDLVertexDescriptor *mdlVertexDescriptor =
//    MTKModelIOVertexDescriptorFromMetal(_mtlVertexDescriptor);
//
//    mdlVertexDescriptor.attributes[VertexAttributePosition].name  = MDLVertexAttributePosition;
//    mdlVertexDescriptor.attributes[VertexAttributeTexcoord].name  = MDLVertexAttributeTextureCoordinate;

    mdlMesh2.vertexDescriptor = mdlVertexDescriptor;

    _mesh2 = [[MTKMesh alloc] initWithMesh:mdlMesh2
                                    device:_device
                                     error:&error];

    if(!_mesh2 || error)
    {
        NSLog(@"Error creating MetalKit mesh %@", error.localizedDescription);
    }
    
    
    // The Quad's texture
    MTKTextureLoader *textureLoader2 = [[MTKTextureLoader alloc] initWithDevice:_device];
    
    NSDictionary *textureLoaderOptions2 =
    @{
      MTKTextureLoaderOptionTextureUsage       : @(MTLTextureUsageShaderRead),
      MTKTextureLoaderOptionTextureStorageMode : @(MTLStorageModePrivate)
      };
    
    _fontTexture = [textureLoader2 newTextureWithName:@"FontAtlas2"
                                            scaleFactor:1.0
                                                 bundle:nil
                                                options:textureLoaderOptions2
                                                  error:&error];
    
    if(!_fontTexture || error)
    {
        NSLog(@"Error creating texture %@", error.localizedDescription);
    }
    ////////////////////////////////
}





- (void)_updateDynamicBufferState
{
    /// Update the state of our uniform buffers before rendering

    _uniformBufferIndex = (_uniformBufferIndex + 1) % kMaxBuffersInFlight;

    _uniformBufferOffset = kAlignedUniformsSize * _uniformBufferIndex;

    _uniformBufferAddress = ((uint8_t*)_dynamicUniformBuffer.contents) + _uniformBufferOffset;
}




- (void)_updateGameState
{
    /// Update any game state before encoding renderint commands to our drawable
    
    // TODO: implement view and movement like Quake
    // Mouse: rotates camera
    // Arrows: translate camera

    Uniforms *uniforms = (Uniforms*)_uniformBufferAddress;

    // Set the projection matrix in the shader
    uniforms->projectionMatrix = _projectionMatrix;
    uniforms->projectionMatrix2 = _projectionMatrix2;
    
    
    // Voor boterzachte beweging gebruiken we natuurlijk continuous keypress
    // We emuleren hier DirectInput door een key_down boolean te gebruiken
    // Pitch
    _modRotX = 0.0;
    _modRotX = -_dY/100; // mouse
     _dY = 0.0;
    
    if (_upkey_down)    _modRotX = 0.05;
    if (_downkey_down)  _modRotX = -0.05;
    
 
    
    // Yaw
    _modRotY = 0.0;
    if (_akey_down || _leftmouse_down)  _modRotY = 0.05;
    if (_dkey_down || _rightmouse_down) _modRotY = -0.05;
    
    // Roll
    _modRotZ = 0.0;
    _modRotZ = _dX/100; // mouse
    _dX = 0.0;
    
    if (_leftkey_down)  _modRotZ = -0.05;
    if (_rightkey_down) _modRotZ = 0.05;
    
 
    

    // Collective
    _modY = 0.0;
    _modY = _dScrollY/100; // mouse
    _dScrollY = 0.0;
    
    if (_wkey_down) _modY = 0.1;
    if (_skey_down) _modY = -0.1;
    
    // Forward
    _modZ = 0.0;
    if (_vkey_down) _modZ = -0.1;
    if (_ckey_down) _modZ = 0.1;
    
    // Sideways
    _modX = 0.0;
    if (_zkey_down) _modX = 0.1;
    if (_xkey_down) _modX = -0.1;
    
    
    
    // Cube
    vector_float3 rotationAxis = {1, 1, 0};
    
    // Should not init here, only update
    //matrix_float4x4 modelMatrix = matrix4x4_rotation(_rotation, rotationAxis);
    
    
    ///////////////////
    // NOTE: Shit, that's it: we should not be recreating the view matrix again every frame:
    // it should be UPDATED every frame. Same goes for the models.
    // The model (and view/cam) matrix stores the model's orientatien in the world.
    // This must only be updated, no recreated here every frame. And don't use Euler angles to rotate.
    // En quaternions garanderen niet dat je geen gimbal lock krijgt. No Euler angles is the trick.
    // Klopt, maar in Direct3D deed je daarnaast ook de view update in een separate pass:
    // m_pd3dDevice->SetTransform( D3DTS_VIEW, &matView );
    // Wij zijn hier elke keer de viewMatrix met de modelMatrix aan het vermenigvuldigen.
    // Zo werkt het niet.
    // View moet waarschijnlijk in een separate pipeline???
    // Je zou kunnen starten met eerst weer een heli model. Als die goed roteert zal je camera dat ook doen.
    // (Je hoeft de eye dan allen in het model te verplaatsen) Heli cam is veel lastiger dan FPS.
    // See: XFile waar we alles al eens gedaan hebben...
    
    
    
    //matrix_float4x4 viewMatrix = matrix4x4_translation(0.0, 0.0, -8.0);

    viewMatrix = matrix_look_at(0.0, 0.0, -9.0, // eye
                                viewMatrix.columns[0][3], viewMatrix.columns[1][3], viewMatrix.columns[2][3],  // center (= at)
                                0.0, 1.0, 0.0); // up


    
    
    // TODO: Solve gimbal lock ////////////////
    // Mod rotation
    // NOTE: als we zo roteren zullen we gimbal lock krijgen. (De assen draaien om).
    // We moeten de camera roteren zoals wanneer we in de heli zaten. Dan kun je onbeperkt helemaal rond zonder dat assen veranderen.
    // De heli (camera) heeft daarvoor zijn eigen matrix en je roteert met quaternions.
    // Voor een FPS view maakt het niet zoveel uit omdat je een begrenzing hebt op de pitch van de camera waardoor
    // gimbal lock niet mogelijk is.
    // Bij een heli zit er zo'n beperking natuurlijk niet.
    // TODO: walking forward must always be in camera z-axis, not world z-axis
    // We need a translation matrix for the camera's coordinate system
    // TODO: These are Euler angles rotations. Strictly no go!!! They cause gimbal lock. /gimbl lok/
    // OKOK: We hebben 6DOF maar we roteren nog om de world axis. What to do?
    // De truc is: je moet eerst het model transleren naar world origin (0,0,0), dan roteren, dan weer
    // terug transleren.
    // En bovendien gebruikten we in XFile.cpp toch gewoon Euler angle rotation. Maar de echte truc is
    // dat je moet dat doen om de model axes waarbij je die model axes elke
    // frame ook moet updaten met de rotatie. En je hebt Gram-Schmidt orthogonalization nodig.
    // Een hoop gedoe dus. Kijk hiervoor in XFile.cpp waar al deze matrix rotatie code aanwezig is.
    // Maar uiteindelijk hebben we ook in XFile.cpp gemakkelijk gedaan en quaternions gebruikt en wel met:
    // D3DXQuaternionRotationYawPitchRoll()
    // De code voor matrices en model axes updates en Gram-Schmidt hebben we wel actief gehouden, zag ik,
    // zodat we ook over konden schakelen op matrix rotatie. En zodat we die axes handy hadden.
    // Maar: quaternions rule!!! Rotation without gimbal lock is much easier achieved.
    // NOTE: de modelMatrix stored de model positie en orientatie:
    // From XFile.cpp:
    // NOTE: m_matFileObjectMatrix stores the 6DOF's (position and orientation) of the heli.
    // 6DOF: 6 Degrees of Freedom (3 linear + 3 angular):
    // In a left-handed coordinate system:
    // | Right.x Up.x  Forward.x 0 |
    // | Right.y Up.y  Forward.y 0 |
    // | Right.z Up.z  Forward.z 0 |
    // | Pos.x   Pos.x Pos.z     1 |
    //
    // See: https://www.youtube.com/watch?v=zc8b2Jo7mno
    // FPS-games gebruiken ook de gimbal sequence die gebruikt wordt in Maya: y->x->z
    // De belangrijkste as eerst: y (yaw or pan); dan de volgende: x (pitch or tilt) [waarbij shooters een
    // beperking in up en down direction aanbrengen om gimbal lock te voor komen]; en als laatste: z (roll
    // or bank) omdat een camera bijna nooit rolled (alleen in Dutch angle shots).
    // Met een heli game mag zo'n beperking er niet zijn: er moet altijd volledige rotatie mogelijk zijn.
    // Dit doe je door te draaien om de model axes. Hiervoor moet je dus de model axes ook elke frame
    // updaten.
    // TODO: also implement the old matrix code that solves gimbal lock and present an option in a menu
    // to use quarternions or matrices.
    
  
    
    
    // Mod rot (matrices)
    // Mmm, in fact we get never get any gimbal lock...
    // 1. Doing rotations around the origin axes gives no true gimbal lock: it only switches controls:
    // e.g. yaw left 90 deg. then roll will have become pitch and pitch roll, from the perspective of the
    // model. From the perspective of the world axis the rotations are correct of course.
    // 2. Here we also get no gimbal lock. At the origin we now always get rotations along the model axes.
    // (which is what we want) but once we move away from the origin, rotations are still around world axes.
    // If we do it like this we also do not get gimbal lock. Rather, the problem is that once we are not
    // at the origin, rotation will still be along origin axes. Even when we try to do that along
    // model axes.
    // Hier moeten we die dubbele truc toepassen: vRight, vUp, vForward every frame updaten; heli naar
    // origin translaten, rotatie don, en dan terug.
    // Nee, die laatste stap deden we niet in XFile.cpp. Dat was een bedachte oplossing die nooit heeft gewerkt.
    // De truc is je model vectoren updaten met de rotatie matrices en dan roteren om die vertices.
    // Maar waarom lukt dat zo niet? De model matrix zou de model orientatie axes toch moeten bevatten?
    // Of werkt het zo: we doen translatie, en dan bevat de matrix niet meer de juiste model vectoren?
    // Check dit.
    // 3. We had our matrices in the wrong order at multiplication. modelMatrix must be first. Remember:
    // matrix multiplication is not commutative. We now always get rotation around the model origin.
    // Problem is that rotation is al messed up. What's going on?
    // We were using the model axis. Now we use the world axes. And all of a sudden we get proper 6DOF heli
    // movement and rotation. Why? Without doing any of the vector update stuff...
    // We must check with prpper model whether tha rotation will not skew the model and rotation will stay correct
    // but it looks good. Euler angle rotation without gimbal lock and around the model origin.
    // It also does not matter whether we do translation before or after the rotation. Matrix order at
    // multiplication does matter of course: modelMatrix first.
    // Who would've thunk? Correct 6DOF heli movement using matrices without gimbal lock, skewing,
    // and correctly rotating around the model axes. Whereas it looks we are always doing rotations
    // around the world axes? Why does this work at all? It never worked!
    // So the trick is using a global modelMatrix (of course) and just making sure matrix multiplication
    // order is correct.
    printf("%f\n", modelMatrix.columns[0][0]);
    printf("%f\n", modelMatrix.columns[0][1]);
    printf("%f\n\n", modelMatrix.columns[0][2]);
    
                     //Method 1. world axes //Method 2. model axes
    rotationAxis.x = 1.0; //modelMatrix.columns[0][0]; //1.0; //
    rotationAxis.y = 0.0; //modelMatrix.columns[0][1]; //0.0; //
    rotationAxis.z = 0.0; //modelMatrix.columns[0][2]; //0.0; //
    matrix_float4x4 rotXMatrix = matrix4x4_rotation(_modRotX, rotationAxis);
    
    rotationAxis.x = 0.0; //modelMatrix.columns[1][0]; //0.0; //
    rotationAxis.y = 1.0; //modelMatrix.columns[1][1]; //1.0; //
    rotationAxis.z = 0.0; //modelMatrix.columns[1][2]; //0.0; //
    matrix_float4x4  rotYMatrix = matrix4x4_rotation(_modRotY, rotationAxis);
    
    rotationAxis.x = 0.0; //modelMatrix.columns[2][0]; //0.0; //
    rotationAxis.y = 0.0; //modelMatrix.columns[2][1]; //0.0; //
    rotationAxis.z = 1.0; //modelMatrix.columns[2][2]; //1.0; //
    matrix_float4x4  rotZMatrix = matrix4x4_rotation(_modRotZ, rotationAxis);
    
                                 // Method 3. modelMatrix first
    modelMatrix = matrix_multiply(modelMatrix, rotXMatrix); // Pitch
    modelMatrix = matrix_multiply(modelMatrix, rotYMatrix); // Yaw
    modelMatrix = matrix_multiply(modelMatrix, rotZMatrix); // Roll
        
        
        // From Xfile.cpp
        // To do rotation with matrices we have to do it like this (and Gram-Schmidt orthogonalization step)
        // Then we have no gimbal lock. But with quaternions is easier.
    //    // update object axes
    //    D3DMath_VectorMatrixMultiply(m_vForward, m_vForward, matRotY);
    //    D3DMath_VectorMatrixMultiply(m_vRight, m_vRight, matRotY);
    //    D3DMath_VectorMatrixMultiply(m_vForward, m_vForward, matRotX);
    //    D3DMath_VectorMatrixMultiply(m_vUp, m_vUp, matRotX);
    //    D3DMath_VectorMatrixMultiply(m_vRight, m_vRight, matRotZ);
    //    D3DMath_VectorMatrixMultiply(m_vUp, m_vUp, matRotZ);
    //
    //    // rotation
    //    D3DMATRIX matRot2, matRotX, matRotY, matRotZ;
    //    D3DUtil_SetRotationMatrix( matRotX, m_vRight, m_fRadsX );
    //    D3DUtil_SetRotationMatrix( matRotY, m_vUp, m_fRadsY );
    //    D3DUtil_SetRotationMatrix( matRotZ, m_vForward, m_fRadsZ );
    //
    //    // Gram-Schmidt orthogonalization algorithm ////////////////////////////////////
    //    // perform base vector regeneration
    //    // (needn't do this every framemove)
    //    // TODO: we are still drifting after several rolls and pitches. Solve it!
    //    // DONE: it was a matrix multiplication order prob, nothing wrong here
    //    m_vForward = Normalize(m_vForward); // just normalize the most important vector
    //    m_vUp = Normalize( m_vUp - ( DotProduct(m_vUp, m_vForward)*m_vForward ) ); // the Gram-Schmidt step
    //    m_vRight = Normalize( CrossProduct(m_vForward, m_vUp) );
    //
    //    // or alternatively:
    //    //m_vRight = Normalize( CrossProduct(m_vUp, m_vForward) );
    //    //m_vUp = Normalize( CrossProduct(m_vForward, m_vRight) );


    
    
  
    // Mod rot (quaternions)
    // From XFile.cpp
//    // Quaternions //////////////////////////////////////////////////////////////////
//    // Yyyyyyyyyyyyoooooooooooo!!!
//    D3DXMATRIX matTrans;
//    D3DXMatrixTranslation( &matTrans, m_fX, m_fY, m_fZ );
//    //D3DXMatrixMultiply(&m_matFileObjectMatrix, &matTrans, &m_matFileObjectMatrix);
//    D3DMath_MatrixMultiply(m_matFileObjectMatrix, matTrans, m_matFileObjectMatrix);    // order is crucial!!!
//
//    D3DXMATRIX matRot;
//    D3DXQUATERNION qRot;
//    D3DXQuaternionRotationYawPitchRoll( &qRot, m_fRadsY, m_fRadsX, m_fRadsZ );
//    D3DXMatrixRotationQuaternion( &matRot, &qRot );
//    //D3DXMatrixMultiply(&m_matFileObjectMatrix, &matRot, &m_matFileObjectMatrix);
//    D3DMath_MatrixMultiply(m_matFileObjectMatrix, matRot, m_matFileObjectMatrix); // order is crucial!!!
    
    // YYYYYYYYYeeeeessssssssss!!!!!!!!!!!!!!!!!
    matrix_float4x4 matTrans = matrix4x4_translation(_modX, _modY, _modZ);
    //
    modelMatrix = matrix_multiply(modelMatrix, matTrans); // order is crucial!!! modelMatrix first otherwise we get collective along world axes
    

    matrix_float4x4 matRot;
    quaternion_float qRot;
    qRot = quaternion_rotation_yaw_pitch_roll(_modRotY, _modRotX, _modRotZ);
    matRot = matrix4x4_from_quaternion(qRot);
    //
    //modelMatrix = matrix_multiply(modelMatrix, matRot); // order is crucial!!!
    
    
    
    
    
    
    // From Xfile.cpp
    // To do rotation with matrices we have to do it like this (and Gram-Schmidt orthogonalization step)
    // Then we have no gimbal lock. But with quaternions is easier.
//    // update object axes
//    D3DMath_VectorMatrixMultiply(m_vForward, m_vForward, matRotY);
//    D3DMath_VectorMatrixMultiply(m_vRight, m_vRight, matRotY);
//    D3DMath_VectorMatrixMultiply(m_vForward, m_vForward, matRotX);
//    D3DMath_VectorMatrixMultiply(m_vUp, m_vUp, matRotX);
//    D3DMath_VectorMatrixMultiply(m_vRight, m_vRight, matRotZ);
//    D3DMath_VectorMatrixMultiply(m_vUp, m_vUp, matRotZ);
//
//     rotation
//    D3DMATRIX matRot2, matRotX, matRotY, matRotZ;
//    D3DUtil_SetRotationMatrix( matRotX, m_vRight, m_fRadsX );
//    D3DUtil_SetRotationMatrix( matRotY, m_vUp, m_fRadsY );
//    D3DUtil_SetRotationMatrix( matRotZ, m_vForward, m_fRadsZ );
//
//    // Gram-Schmidt orthogonalization algorithm ////////////////////////////////////
//    // perform base vector regeneration
//    // (needn't do this every framemove)
//    // TODO: we are still drifting after several rolls and pitches. Solve it!
//    // DONE: it was a matrix multiplication order prob, nothing wrong here
//    m_vForward = Normalize(m_vForward); // just normalize the most important vector
//    m_vUp = Normalize( m_vUp - ( DotProduct(m_vUp, m_vForward)*m_vForward ) ); // the Gram-Schmidt step
//    m_vRight = Normalize( CrossProduct(m_vForward, m_vUp) );
//
//    // or alternatively:
//    //m_vRight = Normalize( CrossProduct(m_vUp, m_vForward) );
//    //m_vUp = Normalize( CrossProduct(m_vForward, m_vRight) );

    
    
    

      // To origin...
//    float modelx = modelMatrix.columns[0][3];
//    float modely = modelMatrix.columns[1][3];
//    float modelz = modelMatrix.columns[2][3];
//    modelMatrix.columns[0][3] = 0.0;
//    modelMatrix.columns[1][3] = 0.0;
//    modelMatrix.columns[2][3] = 0.0;
   
    
    
    //modelMatrix = matrix_multiply(rotXMatrix, modelMatrix); // Pitch
    //modelMatrix = matrix_multiply(rotYMatrix, modelMatrix); // Yaw
    //modelMatrix = matrix_multiply(rotZMatrix, modelMatrix); // Roll
    
    
    
      // ...and back. Jammer, werkt niet.
//    modelMatrix.columns[0][3] = modelx;
//    modelMatrix.columns[1][3] = modely;
//    modelMatrix.columns[2][3] = modelz;
    
    
    // Model translation
    // NOTE: modTrans last (matrix multiplication is not commutative)
    //matrix_float4x4 matTrans = matrix4x4_translation(_modX, _modY, _modZ);
    //modelMatrix = matrix_multiply(modelMatrix, matTrans);
    
    
 
    
    
    
    //viewMatrix = matrix_multiply(viewMatrix, camMatrix);
    
    
    // We need D3DXMatrixLookAtLH to build out viewMatrix
//    D3DXMATRIX* D3DXMatrixLookAtLH(
//      _Inout_       D3DXMATRIX  *pOut,
//      _In_    const D3DXVECTOR3 *pEye,
//      _In_    const D3DXVECTOR3 *pAt,
//      _In_    const D3DXVECTOR3 *pUp
//    );
    // From AAPLMathUtilities.m
//    matrix_float4x4 matrix_look_at(float eyeX, float eyeY, float eyeZ,
//                                  float centerX, float centerY, float centerZ,
//                                  float upX, float upY, float upZ)

    
    
    // Nono
    // strafe
    //viewMatrix.columns[0][0] *= _camX;
    //viewMatrix.columns[0][1] *= _camX;
    //viewMatrix.columns[0][2] *= _camX;
    
    // forward
    //viewMatrix.columns[3][0] *= _camZ;
    //viewMatrix.columns[3][1] *= _camZ;
    //viewMatrix.columns[3][2] *= -_camZ;
    
    
    // Hier gaat het waarschijnlijk fout: deze pipeline zet modelViewMatrix dus tegelijk
    // doen we dus de view en de model matrix. In Direct3D waren die gescheiden. Moeten we dat met shaders
    // doen via een aparte pipeline??? Maar kan dat voor een view (=camera)???
    
    // Set the modelViewMatrix in the shader
    uniforms->modelViewMatrix = matrix_multiply(viewMatrix, modelMatrix);

    _rotation += .0;
    //_rotation += _var;
    
    // No go
    // forward
    //viewMatrix.columns[3][0] *= _camZ;
    //viewMatrix.columns[3][1] *= _camZ;
    //viewMatrix.columns[3][2] *= _camZ;
    
    
    // Set the projection matrix in the shader
    // We want the quad in screen coordinates
    // Mmm, does not work like this. It seems we need a projectionMatrix2 in the shader too.
    // The shaders are only called at the end of this render stage so we cannot update them here.
    // Setting the projection matrix here also affects the Cube. You really need to have all separate
    // variables in the shaders for each model.
    // uniforms->projectionMatrix = _projectionMatrix2;
    // NOTE: we do not longer use modelViewMatrix2 in the shader because we want the quad to have
    // screen coordinates and not be in our world.
    
    
    // Quad
    vector_float3 rotationAxis2 = {1, 1, 0};
    matrix_float4x4 modelMatrix2 = matrix4x4_rotation(_rotation2, rotationAxis2);
    matrix_float4x4 viewMatrix2 = matrix4x4_translation(-1.9, -2.0, -3.5);
    
    // Set the modelViewMatrix2 in the shader
    uniforms->modelViewMatrix2 = matrix_multiply(viewMatrix2, modelMatrix2);
    
    _rotation2 += .00;
    
}


// NOTE: we want our font atlas to have its own model view matrix and not rotate but this seems
// impossble since the rotation is done in the .metal file to all vertices with vertexShader
// using modelViewMatrix. we don't want a shader to do that!!! Shaders should be used for fancy
// stuff, not rotating models.
// Well, it seems that GPU vertex shaders are especially designed for that task. Welcome to the new age.
// We moeten rotatie dus wel in de shader doen maar...:
// TODO: how can we give each model its own matrix? That is, its own vertex shader.
// Moeten we dit in pipelineStateDescriptor doen? Want daar kun je de vertexShader zetten.
//- (void)_updateGameState2
//{
//    /// Update any game state before encoding renderint commands to our drawable
//
//    Uniforms *uniforms = (Uniforms*)_uniformBufferAddress;
//
//    uniforms->projectionMatrix = _projectionMatrix;
//
//    vector_float3 rotationAxis = {1, 1, 0};
//    matrix_float4x4 modelMatrix = matrix4x4_rotation(0.0, rotationAxis);
//    matrix_float4x4 viewMatrix = matrix4x4_translation(0.0, 0.0, -18.0);
//
//    uniforms->modelViewMatrix = matrix_multiply(viewMatrix, modelMatrix);
//
//    //_rotation += .01;
//}



- (void)drawInMTKView:(nonnull MTKView *)view
{
    /// Per frame updates here

    dispatch_semaphore_wait(_inFlightSemaphore, DISPATCH_TIME_FOREVER);

    id <MTLCommandBuffer> commandBuffer = [_commandQueue commandBuffer];
    commandBuffer.label = @"MyCommand";

    __block dispatch_semaphore_t block_sema = _inFlightSemaphore;
    [commandBuffer addCompletedHandler:^(id<MTLCommandBuffer> buffer)
     {
         dispatch_semaphore_signal(block_sema);
     }];

    [self _updateDynamicBufferState];

    // TODO: This should be split out over Cube and Quad
    // Quad should not rotate
    [self _updateGameState];
    

    /// Delay getting the currentRenderPassDescriptor until we absolutely need it to avoid
    ///   holding onto the drawable and blocking the display pipeline any longer than necessary
    MTLRenderPassDescriptor* renderPassDescriptor = view.currentRenderPassDescriptor;

    if(renderPassDescriptor != nil) {

        /// Final pass rendering code here

        id <MTLRenderCommandEncoder> renderEncoder =
        [commandBuffer renderCommandEncoderWithDescriptor:renderPassDescriptor];
        renderEncoder.label = @"MyRenderEncoder";

        
   
        [renderEncoder pushDebugGroup:@"DrawBox"];

        [renderEncoder setFrontFacingWinding:MTLWindingCounterClockwise];
        [renderEncoder setCullMode:MTLCullModeBack];
        [renderEncoder setRenderPipelineState:_pipelineState];
        [renderEncoder setDepthStencilState:_depthState];

        [renderEncoder setVertexBuffer:_dynamicUniformBuffer
                                offset:_uniformBufferOffset
                               atIndex:BufferIndexUniforms];

        [renderEncoder setFragmentBuffer:_dynamicUniformBuffer
                                  offset:_uniformBufferOffset
                                 atIndex:BufferIndexUniforms];
        
        
        // Wireframe
        // NOTE: wireframes draw ugly: the lines are vague and we have 4 quads per cube side
        // What's going on?
        // This: MDLMesh newBoxWithDimensions segments 2
        //[renderEncoder setTriangleFillMode: MTLTriangleFillModeFill];
        //[renderEncoder setTriangleFillMode: MTLTriangleFillModeLines];
        

        
 
        // What's going on? Why are we crashing when we delete this stuff?
        // We are not even drawing the cube anymore. We just do the quad. And crash when we delete
        // this. Why??????
        // It probably had to do with the mdlVertexDescriptor in _loadAssets
        
        
        // Draw the Cube
        // Rotate etc. here instead of by the shaders so we can only rotate this cube.
        // We have to multiply every vertex in the mesh with modelViewMatrix
        // NOTE: we can do that here but it is old style. If you use vertex shaders the GPU will do it.
        // Say goodbye to the Fixed Function Pipeline (FFP). Shaders is the buzzword.
        //
        //vector_float3 rotationAxis = {1, 1, 0};
        //matrix_float4x4 modelMatrix = matrix4x4_rotation(0.0, rotationAxis);
        //matrix_float4x4 viewMatrix = matrix4x4_translation(0.0, 0.0, -18.0);
        //
        //matrix_float4x4 modelViewMatrix = matrix_multiply(viewMatrix, modelMatrix);
        
  
        
        // Set vertex buffer
        for (NSUInteger bufferIndex = 0; bufferIndex < _mesh.vertexBuffers.count; bufferIndex++)
        {
            // TODO: hier moeten we Cube vertices vermenigvuldigen met de modelViewMatrix voor rotatie etc.
            // NONO: we are using the vertex shader for that.
            MTKMeshBuffer *vertexBuffer = _mesh.vertexBuffers[bufferIndex];
            
            if((NSNull*)vertexBuffer != [NSNull null])
            {
                [renderEncoder setVertexBuffer:vertexBuffer.buffer
                                        offset:vertexBuffer.offset
                                       atIndex:bufferIndex];
            }
        }

        // Set the texture
        [renderEncoder setFragmentTexture:_colorMap
                                  atIndex:TextureIndexColor];

        // Draw the mesh
        for(MTKSubmesh *submesh in _mesh.submeshes)
        {
            [renderEncoder drawIndexedPrimitives:submesh.primitiveType
                                      indexCount:submesh.indexCount
                                       indexType:submesh.indexType
                                     indexBuffer:submesh.indexBuffer.buffer
                               indexBufferOffset:submesh.indexBuffer.offset];
        }
    
 
        for (NSUInteger bufferIndex = 0; bufferIndex < _mesh2.vertexBuffers.count; bufferIndex++)
        {
            MTKMeshBuffer *vertexBuffer = _mesh2.vertexBuffers[bufferIndex];
            if((NSNull*)vertexBuffer != [NSNull null])
            {
                [renderEncoder setVertexBuffer:vertexBuffer.buffer
                                        offset:vertexBuffer.offset
                                       atIndex:bufferIndex];
            }
        }
        
        
        // TODO: Quad should not spin
        // It is useless to do that here since the vertex shader does the rotation to all vertices
        //[self _updateGameState2];
        // Will this work?
        // YYYYeeeeessss!!! We simply need this to make vertexShader2 the current shader.
        // But is there really no way to call vertexShader with an extra argument containing the
        // modelViewMatrix of every model??? It does not seem there is. This is the way to do it.
        // So we set _pipelineState2 which uses vertexShader2 which uses modelViewMatrix2 for our Quad model.
        // Behoorlijk omslachtig: twee bestanden moeten met elkaar communiceren.
        // En om de characters in de font atlas texture te verkrijgen zullen we de texture coordinates
        // toch echt wel moeten benaderen. Dat kan in de vertex shader maar je kunt daar bv. al geen global
        // variable gebruiken om door de vertices van de quad te loopen. Lastig.
        // Dit sample: https://developer.apple.com/documentation/metal/synchronization/synchronizing_cpu_and_gpu_work?language=objc
        // lijkt veel op wat wij moeten doen voor de characters. En ook daar worden de vertices toch hier
        // gemanipuleerd, en niet in de shader.
        // Vertex shaders lijken fancy maar zijn in feite lastig om mee te werken. We kunnen niet makkelijk
        // door vertices loopen.
        
        
        /////////////////////////////////////////////////////////////////////////////////
        // TODO: kunnen we de mesh vertices hier manipuleren? Nee, natuurlijk niet de boel zit al in
        // een MTKMesh en die heeft al geen x y z u v etc meer. Hoewel...
        // Vertex data for the current triangles.
        // De sample doet dit waarbij _vertexBuffers een pointer to MTLBuffer is.
        // Maar die sample heeft zelf de vertices erin gezet. Wij doen het met MDLMesh en weten niet
        // wat voor vertex het is.
        //
        // Vertex data for the current triangles.
        // AAPLVertex *currentTriangleVertices = _vertexBuffers[_currentBuffer].contents;
        //
        // Onze _mesh2 is een pointer to MTKMesh en _mesh2.vertexBuffers[0] is een MTKMeshBuffer
        // maar die laatste contained weer een MTLBuffer. Dus de vertices zouden bereikbaar moeten zijn.
        // En met MTKMesh.VertexDescriptor weten we ook wat voor type vertices het zijn.
        //MDLVertexDescriptor
        //MTLVertexDescriptor
        typedef struct
        {
            vector_float3 position;
            vector_float2 texture;
        } QuadVertex;

        
        //for (NSUInteger i = 0; i < 3; i++)
        {
            // Yyyeesss: we can access vertices here!!! Let's first f around with the cube
            // NOTE: our cube had 2 segments. Let's set these to 1 at init so our cube has only 8 vertices.
            // Well, we can get at the vertices here but we really don't know what were doing.
            // Why does the cube not draw properly when we are only slightly modifying the first vertex position
            // And why does vector_float3 report as 4 floats in the debugger???
            // Shit. Getting at vertices is always a hassle. Basically there are 3 options for models:
            // 1. You can build your (simple) model with handmade vertices and then you know what you are doing.
            // 2. You can use functions like MDLMesh newBoxWithDimensions but then we cannot really see the vertices
            // 3. Use an .OBJ file created in Blender or 3dsmax and use a file loader (cf. XFile) and then things get really ugly
            
            QuadVertex *quadVertices = _mesh.vertexBuffers[0].buffer.contents;
            
            vector_float3 pos;
            pos.x = 3.0;
            pos.y = -3.0;
            pos.z = -3.0;
            //quadVertices[0].position = pos; // our first vertex's position should be here at the beginning of the buffer
            
            //vector_float2 tex;
            //tex.x = 3.0;
            //tex.y = -3.0;
            //quadVertices[0].texture = tex;
        }
        /////////////////////////////////////////////////////////////////////////////////
        
        
        
        
        // Set pipeline for the Quad
        [renderEncoder setRenderPipelineState:_pipelineState2];
        
        
        // Draw the Quad
        // Set vertex buffer
        for (NSUInteger bufferIndex = 0; bufferIndex < _mesh2.vertexBuffers.count; bufferIndex++)
        {
            MTKMeshBuffer *vertexBuffer = _mesh2.vertexBuffers[bufferIndex];
            if((NSNull*)vertexBuffer != [NSNull null])
            {
                [renderEncoder setVertexBuffer:vertexBuffer.buffer
                                        offset:vertexBuffer.offset
                                       atIndex:bufferIndex];
            }
        }

        // Set the texture
        [renderEncoder setFragmentTexture:_fontTexture
                                  atIndex:TextureIndexColor];

        // Draw the mesh
        for(MTKSubmesh *submesh in _mesh2.submeshes)
        {
            [renderEncoder drawIndexedPrimitives:submesh.primitiveType
                                      indexCount:submesh.indexCount
                                       indexType:submesh.indexType
                                     indexBuffer:submesh.indexBuffer.buffer
                               indexBufferOffset:submesh.indexBuffer.offset];
        }
        
        
        

        [renderEncoder popDebugGroup];

        [renderEncoder endEncoding];

        // Present
        [commandBuffer presentDrawable:view.currentDrawable];
    }

    // And Commit
    [commandBuffer commit];
}



- (void)mtkView:(nonnull MTKView *)view drawableSizeWillChange:(CGSize)size
{
    /// Respond to drawable size or orientation changes here

    float aspect = size.width / (float)size.height;
    _projectionMatrix = matrix_perspective_right_hand(65.0f * (M_PI / 180.0f), aspect, 0.1f, 100.0f);
    
    // Quad
    _projectionMatrix2 = matrix_perspective_right_hand(65.0f * (M_PI / 180.0f), aspect, 0.1f, 100.0f);
}





///////////////////////////////////////////
#pragma mark Matrix Math Utilities

//matrix_float4x4 matrix4x4_translation(float tx, float ty, float tz)
//{
//    return (matrix_float4x4) {{
//        { 1,   0,  0,  0 },
//        { 0,   1,  0,  0 },
//        { 0,   0,  1,  0 },
//        { tx, ty, tz,  1 }
//    }};
//}

//static matrix_float4x4 matrix4x4_rotation(float radians, vector_float3 axis)
//{
//    axis = vector_normalize(axis);
//    float ct = cosf(radians);
//    float st = sinf(radians);
//    float ci = 1 - ct;
//    float x = axis.x, y = axis.y, z = axis.z;
//
//    return (matrix_float4x4) {{
//        { ct + x * x * ci,     y * x * ci + z * st, z * x * ci - y * st, 0},
//        { x * y * ci - z * st,     ct + y * y * ci, z * y * ci + x * st, 0},
//        { x * z * ci + y * st, y * z * ci - x * st,     ct + z * z * ci, 0},
//        {                   0,                   0,                   0, 1}
//    }};
//}

matrix_float4x4 matrix_perspective_right_hand(float fovyRadians, float aspect, float nearZ, float farZ)
{
    float ys = 1 / tanf(fovyRadians * 0.5);
    float xs = ys / aspect;
    float zs = farZ / (nearZ - farZ);

    return (matrix_float4x4) {{
        { xs,   0,          0,  0 },
        {  0,  ys,          0,  0 },
        {  0,   0,         zs, -1 },
        {  0,   0, nearZ * zs,  0 }
    }};
}


//matrix_float4x4 matrix_look_at(float eyeX, float eyeY, float eyeZ,
//                               float centerX, float centerY, float centerZ,
//                               float upX, float upY, float upZ)
//{
//    vector_float3 eye = vector_make(eyeX, eyeY, eyeZ);
//    vector_float3 center = vector_make(centerX, centerY, centerZ);
//    vector_float3 up = vector_make(upX, upY, upZ);
//
//    vector_float3 z = vector_normalize(eye - center);
//    vector_float3 x = vector_normalize(vector_cross(up, z));
//    vector_float3 y = vector_cross(z, x);
//    vector_float3 t = vector_make(-vector_dot(x, eye), -vector_dot(y, eye), -vector_dot(z, eye));
//
//    return matrix_make(x.x, y.x, z.x, 0,
//                       x.y, y.y, z.y, 0,
//                       x.z, y.z, z.z, 0,
//                       t.x, t.y, t.z, 1);
//}



@end
