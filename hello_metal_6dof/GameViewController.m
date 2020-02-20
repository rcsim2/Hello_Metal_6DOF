//
//  GameViewController.m
//  hello_metal_6dof
//
//  Created by Rik Goossens on 11/02/2020.
//  Copyright © 2020 cgdemy.com. All rights reserved.
//

#import "GameViewController.h"
#import "Renderer.h"

//#import "Events.h"
//#include <Carbon/Carbon.h>


// No beep
// No go
// Kondrak code
// window listener (close and move events)
//@interface MyWindowListener : NSResponder<NSWindowDelegate>
//-(void) listen:(NSApplication *) data;
//-(void) windowDidMove:(NSNotification *) aNotification;
//-(BOOL) windowShouldClose:(id) sender;
//@end

//static MyWindowListener *windowListener = nil;





// No beeps: Yeeesss!!!
// Have to subclass MTKView, handle keydown in the subclass and must set MTKViewSub in Storyboard
@implementation MTKViewSub

- (void)keyDown:(NSEvent *)theEvent {
    NSLog(@"keyDown Detected");
}

- (BOOL)acceptsFirstResponder
{
    return YES;
}

@end




@implementation GameViewController
{
    //MTKView *_view;
    MTKViewSub *_view;

    Renderer *_renderer;
    
    ///////////////
    id keyMonitor;
    id keyMonitor2;
    
    id mouseMonitor;
    
    NSPoint previousLoc;
    bool firstMouse;
}




- (void)viewDidLoad
{
    [super viewDidLoad];

    //_view = (MTKView *)self.view;
    _view = (MTKViewSub *)self.view;

    _view.device = MTLCreateSystemDefaultDevice();

    if(!_view.device)
    {
        NSLog(@"Metal is not supported on this device");
        self.view = [[NSView alloc] initWithFrame:self.view.frame];
        return;
    }

    _renderer = [[Renderer alloc] initWithMetalKitView:_view];

    [_renderer mtkView:_view drawableSizeWillChange:_view.bounds.size];

    _view.delegate = _renderer;
    
    
    // FPS
    _view.preferredFramesPerSecond = 60;
    
    // No beeps
    // Try to get no beeps without subclassing MTKView
    // No go
    // See: https://www.oipapio.com/question-4624151
    // See: https://stackoverflow.com/questions/34456590/handling-input-events-in-a-metalkit-application/60300596#60300596
    // Here we do have a pointer to GameViewController.
    // But how can it work? He's installing a handler for the window whereas we should be doing that for
    // the view. The view's lack of a handler is causing the beeps.
    //GameViewController *myViewController = self;
    //[[[myViewController view] window ] makeFirstResponder:myViewController];
  
    
    
    
    // No beeps on keydown
    // Code from Kondrak. Alas, no go.
    // Get pointer to window
    //_window = self.view.window;
//    windowListener = [[MyWindowListener alloc] init];
//    NSApplication *app = [NSApplication sharedApplication];
//    [windowListener listen:app];
    //[app setActivationPolicy:NSApplicationActivationPolicyRegular];
    
    
    
    
    // Mouse input
    ///////////////////////////////////////////
    // Not necessary and did not work: we only get mouse down.
    // But we need continuous mousemove handler.
    //[self.view.window setAcceptsMouseMovedEvents:YES];
    //[[self.view window] acceptsMouseMovedEvents];
    //[_view setAcceptsTouchEvents:YES];
    //[self.view allowedTouchTypes:YES];
    //[self.view setAcceptsTouchEvents:YES];
    
//    NSTrackingAreaOptions options = (NSTrackingActiveAlways | NSTrackingInVisibleRect |
//                             NSTrackingMouseEnteredAndExited | NSTrackingMouseMoved);
//
//    NSTrackingArea *area = [[NSTrackingArea alloc] initWithRect:[self.view bounds]
//                                                        options:options
//                                                          owner:self
//                                                       userInfo:nil];
//
    // Yyyesss!!!
    // But note the construction: we are handling it here instead of in
    // - (void)mouseMoved:(NSEvent *)event
    // Why here???
    // That is because our view is a MTKView, not a NSView. MTKView does not handle mouse and key events
    // by default.
    // NOTE: we have now subclassed MTKView and then we can use handlers. But just leave this here and
    // only handle keydown in the subclass to get rid of the beeps.
    // TODO: We get x 0-1366 and y 0-768 en op basis daarvan kunnen we de camera roteren maar we moeten de
    // camera blijven roteren als de muis tegen de randen aan zit. Op camRotX (pitch) mag natuurlijk wel
    // een begrenzing onder en boven zitten, maar voor camRotY (yaw) moet de player helemaal rond kunnen
    // blijven kijken.
    // We moeten dus de als de muis tegen de rand zit zorgen dat we de cam blijven roteren.
    // We moeten dus niet roteren op basis van screen coordinates maar op basis van mouse movement (is ook trackpad movement):
    // Zoiets:
    //private Point previousLocation;
    //
    //private void Player_MouseMove(object sender, MouseEventArgs e)
    //{
    //    int differenceX, differenceY;
    //    differenceX = e.X - previousLocation.X;
    //    differenceY = e.Y - previousLocation.Y;
    //    previousLocation = e.Location;
    //}
    //
    // Dit werkt maar nog steeds is het zo dat als we tegen de rand komen het roteren ophoudt.
    // WE MOETEN DE TRACKPAD COORDINATEN GEBRUIKEN IPV DE SCREEN COORDINATEN.
    // We moeten de mouse X.Y gebruiken (de rollers) net als DirectInput die aan ons gaf.
    // Maar hoe doe je dat dan met de muis???
    // Kijk naar Kondrak in vk_imp.m bij RW_IN_Move
    // Of hier: https://learnopengl.com/Getting-started/Camera
    // We moeten de mouse capturen zodat ie niet meer zichtbaar is en in het centrum blijft en inderdaad met
    // mouse deltas werken.
    // Zo verberg je de mouse pointer maar het probleem blijft natuurlijk.
    //[NSCursor hide];
    // We moeten de mouse pointer capturen zodat ie altijd in het centrum blijft staan:
    // Capturing a cursor means that, once the application has focus, the mouse cursor stays within
    // the center of the window (unless the application loses focus or quits). We can do this with one
    // simple configuration call:
    // glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    // After this call, wherever we move the mouse it won't be visible and it should not leave the window.
    // This is perfect for an FPS camera system.
    // We hebben raw mouse input data nodig (the mouse rollers). In Windows kan dat met DirectInput of
    // WM_INPUT (See: https://docs.microsoft.com/en-us/windows/win32/dxtecharts/taking-advantage-of-high-dpi-mouse-movement)
    // Maar in Mac? Hiermee?
//    from Quartz.CoreGraphics import (
//    CGEventCreate,
 //   CGEventCreateMouseEvent,
//    CGEventGetLocation,
//    CGEventPost,
//    CGEventSetIntegerValueField,
//    kCGEventMouseMoved,
//    kCGHIDEventTap,
//    kCGMouseEventDeltaX, // we need this!!!!!
//    kCGMouseEventDeltaY,
//    )
    // Kondrak gebruikt deze: NSEventTypeMouseMoved
    // Also see: deltaX en deltaY
    
    //previousLoc.x = 1366/2;
    //previousLoc.y = 768/2;
    
    //firstMouse = true;
    
 
//    mouseMonitor = [NSEvent addLocalMonitorForEventsMatchingMask:NSEventMaskCursorUpdate handler:^(NSEvent *event) {
//
//        NSLog(@"Cursor update:");
//
//        return event;
//    }];
    
    mouseMonitor = [NSEvent addLocalMonitorForEventsMatchingMask:NSEventMaskScrollWheel handler:^(NSEvent *event) {
        //CGFloat dScrollY;
        self->_renderer.dScrollY = event.scrollingDeltaY; //event.deltaZ; // 0 for most scroll wheel and mouse events
        NSLog(@"dScrollY = %f", self->_renderer.dScrollY);
        
        //self->_renderer.modY += dScrollY/100; // Collective
    
        return event;
    }];

    mouseMonitor = [NSEvent addLocalMonitorForEventsMatchingMask:NSEventMaskMouseMoved handler:^(NSEvent *event) {
    //mouseMonitor = [NSEvent addLocalMonitorForEventsMatchingMask:NSEventMaskDirectTouch handler:^(NSEvent *event) {
        
        
        // This gives screen coordinates. Useless in this case.
        //NSPoint mouseLoc;
        //mouseLoc = [NSEvent mouseLocation]; //get current mouse position
        
        //NSLog(@"Mouse location:");
        //NSLog(@"x = %f",  mouseLoc.x);
        //NSLog(@"y = %f",  mouseLoc.y);
        
        
        // Yyyeesssss!!!
        // deltaX and deltaY give us the real mouse x,y rollers
        // NOTE: deltaZ is the mouse scroll wheel but that is typically 0
        //CGFloat dX, dY;
        self->_renderer.dX = event.deltaX;
        self->_renderer.dY = event.deltaY;
       
        NSLog(@"dX = %f", self->_renderer.dX);
        NSLog(@"dY = %f", self->_renderer.dY);
        
       
        
        
        // prevent initial cam jerk
        //if(self->firstMouse) // initially set to true
        //{
        //    self->previousLoc = mouseLoc;
        //    self->firstMouse = false;
        //}
        
        //float deltaX, deltaY;
        //deltaX = mouseLoc.x - self->previousLoc.x;
        //deltaY = mouseLoc.y - self->previousLoc.y;
        //self->previousLoc = mouseLoc;
        
        // This does not allow is to rotate the cam further when the mouse has reached the window edge.
        //self->_renderer.camRotX -= deltaY/100; //-(mouseLoc.y - 768/2) / 768;
        //self->_renderer.camRotY += deltaX/100; //(mouseLoc.x - 1366/2) / 1366;
        
        
        // This does
        //self->_renderer.camRotX += dY/100;
        //self->_renderer.camRotY += dX/100;
        //self->_renderer.modRotZ = dX/100; // Pitch
        //self->_renderer.modRotX += dY/100; // Roll
        
        
        
        
        
        
        // Reset mouse to centre of screen?
        //NSPoint centerLoc = {1366/2 , 768/2};
        //CGDisplayMoveCursorToPoint(CGMainDisplayID(), centerLoc);
        
        return event;
    }];
    
    
    mouseMonitor = [NSEvent addLocalMonitorForEventsMatchingMask:NSEventMaskLeftMouseDown handler:^(NSEvent *event) {
        
        NSPoint mouseLoc;
        mouseLoc = [NSEvent mouseLocation]; //get current mouse position

        //NSLog(@"Mouse down:");
        //NSLog(@"x = %f",  mouseLoc.x);
        //NSLog(@"y = %f",  mouseLoc.y);
        
        self->_renderer.leftmouse_down = true;

        //self->_renderer.modRotY += 0.05; // Yaw
        
        //self->_renderer.camRotX = -(mouseLoc.y - 768/2) / 768;
        //self->_renderer.camRotY = (mouseLoc.x - 1366/2) / 1366;
        
        return event;
    }];
    
    
    mouseMonitor = [NSEvent addLocalMonitorForEventsMatchingMask:NSEventMaskLeftMouseUp handler:^(NSEvent *event) {
       
        self->_renderer.leftmouse_down = false;
        
        return event;
    }];
    
    
    mouseMonitor = [NSEvent addLocalMonitorForEventsMatchingMask:NSEventMaskRightMouseDown handler:^(NSEvent *event) {
        
        NSPoint mouseLoc;
        mouseLoc = [NSEvent mouseLocation]; //get current mouse position

        //NSLog(@"Mouse down:");
        //NSLog(@"x = %f",  mouseLoc.x);
        //NSLog(@"y = %f",  mouseLoc.y);
        

        self->_renderer.rightmouse_down = true;
        
        //self->_renderer.modRotY -= 0.05; // Yaw
        
        //self->_renderer.camRotX = -(mouseLoc.y - 768/2) / 768;
        //self->_renderer.camRotY = (mouseLoc.x - 1366/2) / 1366;
        
        return event;
    }];
    
    
    mouseMonitor = [NSEvent addLocalMonitorForEventsMatchingMask:NSEventMaskRightMouseUp handler:^(NSEvent *event) {
       
        self->_renderer.rightmouse_down = false;
        
        return event;
    }];
    

    
    
    // Keyboard input
    //////////////
    // NOTE: This is event based repeated key down. For games we need hardware-based low-level continuous
    // keyboard handling like DirectInput which polls the keyboard.
    // DONE: we solved this by using a bool that is set to false on keyup.
    // TODO: get rid of the beeps.
    keyMonitor = [NSEvent addLocalMonitorForEventsMatchingMask:NSEventMaskKeyDown handler:^(NSEvent *event) {

        unichar character = [[event characters] characterAtIndex:0];
        switch (character) {
            case NSUpArrowFunctionKey:
                NSLog(@"up arrow down");
                //self->_renderer.camZ += .5;
                self->_renderer.upkey_down = true;
                break;
            case NSDownArrowFunctionKey:
                NSLog(@"down arrow down");
                //self->_renderer.camZ -= .5;
                self->_renderer.downkey_down = true;
                break;
            case NSLeftArrowFunctionKey:
                NSLog(@"left arrow down");
                //self->_renderer.var = -0.02;
                //self->_renderer.camX += .5;
                self->_renderer.leftkey_down = true;
                break;
            case NSRightArrowFunctionKey:
                NSLog(@"right arrow dowm");
                // FUCK: no way to access that shit
                // ObjC sucks.
                //g_rot += 1.0;
                //_renderer._rotation += 2.0;
                //self->_renderer.var = 0.02;
                //self->_renderer.camX -= .5;
                self->_renderer.rightkey_down = true;
                break;
            
            case 'w':
                NSLog(@"w down");
                self->_renderer.wkey_down = true;
                break;
            case 's':
                NSLog(@"s down");
                self->_renderer.skey_down = true;
                break;
            case 'a':
                NSLog(@"a down");
                self->_renderer.akey_down = true;
                break;
            case 'd':
                NSLog(@"d down");
                self->_renderer.dkey_down = true;
                break;
                
            case 'z':
                NSLog(@"z down");
                self->_renderer.zkey_down = true;
                break;
            case 'x':
                NSLog(@"x down");
                self->_renderer.xkey_down = true;
                break;
            case 'c':
                NSLog(@"c down");
                self->_renderer.ckey_down = true;
                break;
            case 'v':
                NSLog(@"v down");
                self->_renderer.vkey_down = true;
                break;
                
            default:
                break;
        }
        return event;
    }];
    
    
    //////////////
    keyMonitor2 = [NSEvent addLocalMonitorForEventsMatchingMask:NSEventMaskKeyUp handler:^(NSEvent *event) {

        unichar character = [[event characters] characterAtIndex:0];
        switch (character) {
            case NSUpArrowFunctionKey:
                NSLog(@"up arrow up");
                self->_renderer.upkey_down = false;
                break;
            case NSDownArrowFunctionKey:
                NSLog(@"down arrow up");
                self->_renderer.downkey_down = false;
                break;
            case NSLeftArrowFunctionKey:
                NSLog(@"left arrow up");
                //self->_renderer.var = 0.0;
                self->_renderer.leftkey_down = false;
                break;
            case NSRightArrowFunctionKey:
                NSLog(@"right arrow up");
                // FUCK: no way to access that shit
                // ObjC sucks.
                //g_rot += 1.0;
                //_renderer._rotation += 2.0;
                //self->_renderer.var = 0.0;
                self->_renderer.rightkey_down = false;
                break;
                
            case 'w':
                NSLog(@"w up");
                self->_renderer.wkey_down = false;
                break;
            case 's':
                NSLog(@"s up");
                self->_renderer.skey_down = false;
                break;
            case 'a':
                NSLog(@"a up");
                self->_renderer.akey_down = false;
                break;
            case 'd':
                NSLog(@"d up");
                self->_renderer.dkey_down = false;
                break;
                
            case 'z':
                NSLog(@"z up");
                self->_renderer.zkey_down = false;
                break;
            case 'x':
                NSLog(@"x up");
                self->_renderer.xkey_down = false;
                break;
            case 'c':
                NSLog(@"c up");
                self->_renderer.ckey_down = false;
                break;
            case 'v':
                NSLog(@"v up");
                self->_renderer.vkey_down = false;
                break;
                
            default:
                break;
        }
        return event;
    }];
}




// No beeps
//- (BOOL)acceptsFirstResponder {
//    return YES;
//}
//
//- (BOOL)performKeyEquivalent:(NSEvent *)event {
//    return YES;
//}
//
//- (void)keyDown:(NSEvent *)event {
//    //return nil;
//}



// This does not work. Why?
// It is because the View in Game View Controller is not a NSView but a MTKView which does not
// handle mouse and key events, so overriding them here is useless. We have to use
// addLocalMonitorForEventsMatchingMask to install a handler.
// Or subclass MTKView
//- (void)mouseMoved:(NSEvent *)theEvent
//{
//    //NSPoint locationInView = [self.view convertPoint:[event locationInWindow]
//    //                                   fromView:nil];
//
//    NSPoint mouseLoc;
//    mouseLoc = [NSEvent mouseLocation]; //get current mouse position
//
//    NSLog(@"Mouse location2:");
//    NSLog(@"x = %f",  mouseLoc.x);
//    NSLog(@"y = %f",  mouseLoc.y);
//
//}



// No beeps
// No go: have to subclass MTKView and handle keydown there
- (void)keyDown:(NSEvent *)theEvent {
    NSLog(@"keyDown Detected");
}

- (BOOL)acceptsFirstResponder
{
    return YES;
}


/* One of many touch event handling methods. */
//- (void)touchesBeganWithEvent:(NSEvent *)ev {
//   NSSet *touches = [ev touchesMatchingPhase:NSTouchPhaseBegan inView:self];
//
//   for (NSTouch *touch in touches) {
//      /* Once you have a touch, getting the position is dead simple. */
//      NSPoint fraction = touch.normalizedPosition;
//      NSSize whole = touch.deviceSize;
//      NSPoint wholeInches = {whole.width / 72.0, whole.height / 72.0};
//      NSPoint pos = wholeInches;
//      pos.x *= fraction.x;
//      pos.y *= fraction.y;
//      NSLog(@"%s: Finger is touching %g inches right and %g inches up "
//            @"from lower left corner of trackpad.", __func__, pos.x, pos.y);
//   }
//}


@end









// ----------------------

//@implementation MyWindowListener
//- (void)listen:(NSApplication *)data
//{
//    // TODO: need pointer to our hello_metal_6dof windows created in Storyboard
//    // We want to use this code to get no beeps in our window at keypress
//    // But Kondrak has created the window programmatically. In this app it is created
//    // by Storyboard. How to get a pointer?
//    NSWindow *window = [[NSApplication sharedApplication] mainWindow];
//
//    if ([window delegate] == nil) {
//        [window setDelegate:self];
//    }
//
//    [window setAcceptsMouseMovedEvents:YES];
//    [window setNextResponder:self];
//    [[window contentView] setNextResponder:self];
//}
//
//- (void)windowDidMove:(NSNotification *) aNotification
//{
//    //ri.Cvar_Set("vid_xpos", va("%d", (int)[window frame].origin.x))->modified = false;
//    //ri.Cvar_Set("vid_ypos", va("%d", (int)[window frame].origin.y))->modified = false;
//}
//
//- (BOOL)windowShouldClose:(id)sender
//{
//    //in_state->Quit_fp();
//    return YES;
//}
//
//// these empty functions are necessary so that there's no beeping sound when pressing keys
//- (void)flagsChanged:(NSEvent *)theEvent {}
//- (void)keyDown:(NSEvent *)theEvent { NSLog(@"Listener key down:"); }
//- (void)keyUp:(NSEvent *)theEvent {}
//- (void)doCommandBySelector:(SEL)aSelector {}
//@end
