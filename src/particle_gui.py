import pathlib
import time
import os
import sys
from tkinter import *

##################################################
# Initialize C++ bindings
##################################################

import py_pbd_simulation as pbd
# this is all what is needed to have access to exposed c++ 

##################################################
# Class managing the UI
##################################################

class ParticleUI :
    def __init__(self) :
        # create drawing context
        self.context = pbd.Context() # python is responsible of the destruction of the context !
        self.width = 1000
        self.height = 1000

        # physical simulation will work in [-world_x_max,world_x_max] x [-world_y_max,world_y_max]
        self.world_x_max = 10 
        self.world_y_max = 10 
        # WARNING : the mappings assume world bounding box and canvas have the same ratio !

        self.window = Tk()

        # create simulation context...
        self.canvas = Canvas(self.window,width=self.width,height=self.height)
        self.canvas.pack()

        # Initialize the scene
        #Add water
        #self.addWater((-10,0),(10,0))
        
        self.addPlan((-10,9),(-8,-8))
        self.addPlan((-10,-8),(10,-8))
        self.addPlan((8,-8),(10,9))

        #self.addParticle((0,8), 2.0, 10.0, (1.0, 2.0), "orange")
        #self.addParticle((0,-10), 4.0, 2.0, (0.0, 0.0), "orange")
        
        # Initialize Mouse and Key events
        self.canvas.bind("<Button-1>", lambda event: self.mouseCallback(event))
        self.window.bind("<Key>", lambda event: self.keyCallback(event)) # bind all key
        self.window.bind("<Escape>", lambda event: self.enterCallback(event)) 
        # bind specific key overide default binding

    def launchSimulation(self) :
        # launch animation loop
        self.animate()
        # launch UI event loop
        self.window.mainloop()

    def animate(self) :
        """ animation loop """
        # APPLY PHYSICAL UPDATES HERE !
        for i in range(6) :
            self.context.updatePhysicalSystem(0.016/6.0, 1) # can be called more than once..., just devise dt
        for i in range(self.context.num_particles()):
            # TODO Update particle display coordinate
            particle = self.context.particle(i)
            draw_id = []
            draw_id.append(particle.draw_id)
            if particle.radius > 0.0:
                pmin = (particle.position.x - particle.radius, particle.position.y - particle.radius)
                pmax = (particle.position.x + particle.radius, particle.position.y + particle.radius)
                self.canvas.coords(particle.draw_id, *self.worldToView(pmin), * self.worldToView(pmax))
            # print("Update code to display particules !")
            # print(" - Use the function coord from Tk.Canvas to update the bounding box of displayed ellipses corresponding to parameters")
            # print(" - Screen coordinates can be combuted from world coordinates using the methode worldToView")
            # print(" - worldToView return a tuple, tuple expension * can be used to generate list of function parameters")
            # END TODO
        self.window.update()
        self.window.after(16, self.animate)

    # Conversion from world space (simulation) to display space (tkinter canvas)
    def worldToView(self, world_pos) :
        return ( self.width *(0.5 + (world_pos[0]/self.world_x_max) * 0.5),
                 self.height *(0.5 - (world_pos[1]/self.world_y_max) * 0.5)) 
    def viewToWorld(self, view_pos) :
        return ( self.world_x_max * 2.0 * (view_pos[0]/self.width - 0.5) ,
                 self.world_y_max * 2.0 * (0.5-view_pos[1]/self.height))  

    def addParticle(self, world_pos, radius, mass, velocity, color) :
        # min max bounding box in view coordinates, will be propertly initialized 
        # in the canvas oval after the first call to animate
        #b_min = self.worldToView( (x_world-radius,y_world-radius) )
        #b_max = self.worldToView( (x_world+radius,y_world+radius) )
        draw_id = self.canvas.create_oval(0,0,0,0,fill=color)
        # TODO add particle in C++ context
        self.context.addParticle(pbd.Vec2(*world_pos), radius, mass, pbd.Vec2(*velocity), draw_id)
        # print("Update code to add particules!")
        # print("  - You will also need a function on c++ side")
        # print("  - For a C++ struct, it is possible to define a binding init function even in absence of constructor, simply give as template parameters the types of the structure attributs")
        # END TODO

    def getDrawIds(self):
        draw_ids = []
        for widget in self.canvas.winfo_children():
            draw_ids.append(widget.winfo_id())

    def addPlan(self, coord1, coord2):
        # Convert world coordinates to view coordinates
        view_coord1 = self.worldToView(coord1)
        view_coord2 = self.worldToView(coord2)

        # Create a polygon that represents the area under the line
        points = [view_coord1[0], self.height, view_coord1[0], view_coord1[1], view_coord2[0], view_coord2[1], view_coord2[0], self.height]
        self.canvas.create_polygon(points, fill='#836953')

        # Add the plan to the context
        self.context.addPlan(pbd.Vec2(*coord1), pbd.Vec2(*coord2))
    
    def addWater(self, coord1, coord2):
        # Convert world coordinates to view coordinates
        view_coord1 = self.worldToView(coord1)
        view_coord2 = self.worldToView(coord2)

        # Create a polygon that represents the area under the line
        points = [view_coord1[0], self.height, view_coord1[0], view_coord1[1], view_coord2[0], view_coord2[1], view_coord2[0], self.height]
        self.canvas.create_polygon(points, fill='#AEC6CF')

        # Add the plan to the context
        self.context.addWater(pbd.Vec2(*coord1), pbd.Vec2(*coord2))


    # All mouse and key callbacks

    def mouseCallback(self, event):
        self.addParticle(self.viewToWorld((event.x,event.y)), 0.2, 1.0, (0.0, 0.0), "red")
    
    def keyCallback(self, event):
        print(repr(event.char))
    def enterCallback(self, event):
        self.window.destroy()


gui = ParticleUI()
gui.launchSimulation()

