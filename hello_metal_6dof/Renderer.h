//
//  Renderer.h
//  hello_metal_6dof
//
//  Created by Rik Goossens on 11/02/2020.
//  Copyright © 2020 cgdemy.com. All rights reserved.
//

#import <MetalKit/MetalKit.h>



// Our platform independent renderer class.   Implements the MTKViewDelegate protocol which
//   allows it to accept per-frame update and drawable resize callbacks.
@interface Renderer : NSObject <MTKViewDelegate>

-(nonnull instancetype)initWithMetalKitView:(nonnull MTKView *)view;

// We use properties to allow these vars to be accessed in GameViewController
@property (readwrite, nonatomic) float var;

@property (readwrite, nonatomic) float modX;
@property (readwrite, nonatomic) float modY;
@property (readwrite, nonatomic) float modZ;

@property (readwrite, nonatomic) float modRotX;
@property (readwrite, nonatomic) float modRotY;
@property (readwrite, nonatomic) float modRotZ;


@property (readwrite, nonatomic) float camX;
@property (readwrite, nonatomic) float camY;
@property (readwrite, nonatomic) float camZ;

@property (readwrite, nonatomic) float camRotX;
@property (readwrite, nonatomic) float camRotY;
@property (readwrite, nonatomic) float camRotZ;


// Mouse
@property (readwrite, nonatomic) bool leftmouse_down;
@property (readwrite, nonatomic) bool rightmouse_down;

@property (readwrite, nonatomic) CGFloat dX;
@property (readwrite, nonatomic) CGFloat dY;
@property (readwrite, nonatomic) CGFloat dScrollY;


// Keys
@property (readwrite, nonatomic) bool upkey_down;
@property (readwrite, nonatomic) bool downkey_down;
@property (readwrite, nonatomic) bool leftkey_down;
@property (readwrite, nonatomic) bool rightkey_down;

@property (readwrite, nonatomic) bool wkey_down;
@property (readwrite, nonatomic) bool skey_down;
@property (readwrite, nonatomic) bool akey_down;
@property (readwrite, nonatomic) bool dkey_down;

@property (readwrite, nonatomic) bool zkey_down;
@property (readwrite, nonatomic) bool xkey_down;
@property (readwrite, nonatomic) bool ckey_down;
@property (readwrite, nonatomic) bool vkey_down;






@end


