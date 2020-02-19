//
//  AppDelegate.m
//  hello_metal_6dof
//
//  Created by Rik Goossens on 11/02/2020.
//  Copyright © 2020 cgdemy.com. All rights reserved.
//

#import "AppDelegate.h"

@interface AppDelegate ()

@end

@implementation AppDelegate

- (void)applicationDidFinishLaunching:(NSNotification *)aNotification {
    // Insert code here to initialize your application
    
    // No beeps
    // Try to get no beeps without subclassing MTKView
    // No go
    // See: https://www.oipapio.com/question-4624151
    // See: https://stackoverflow.com/questions/34456590/handling-input-events-in-a-metalkit-application/60300596#60300596
    // Cannot make it work. How to get a pointer to GameViewController from here?
    // And how can it work? He's installing a handler for the window whereas we should be doing that for
    // the view. The view's lack of a handler is causing the beeps.
    //[[ [myViewController view] window ] makeFirstResponder:myViewController];
 
  
}

- (void)applicationWillTerminate:(NSNotification *)aNotification {
    // Insert code here to tear down your application
}

- (BOOL)applicationShouldTerminateAfterLastWindowClosed:(NSApplication *)sender {
    return YES;
}

@end
