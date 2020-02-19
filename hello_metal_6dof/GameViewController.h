//
//  GameViewController.h
//  hello_metal_6dof
//
//  Created by Rik Goossens on 11/02/2020.
//  Copyright © 2020 cgdemy.com. All rights reserved.
//

#import <Cocoa/Cocoa.h>
#import <Metal/Metal.h>
#import <MetalKit/MetalKit.h>
#import "Renderer.h"

// Our macOS view controller.
@interface GameViewController : NSViewController

// Pointer to the view's window
@property (readwrite, nonatomic) NSWindow* window;

@end


// No beeps
@interface MTKViewSub : MTKView

@end

