# Script for moving all the bodies in the nested child occurrences into the target occurrence

import adsk.core, adsk.fusion, adsk.cam, traceback

from typing import List

# Global list to keep all event handlers in scope.
# This is only needed with Python.
handlers = []

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        design.designType = adsk.fusion.DesignTypes.DirectDesignType

        # Get the CommandDefinitions collection.
        cmdDefs = ui.commandDefinitions

        # Create a button command definition.
        buttonSample = cmdDefs.addButtonDefinition('ACDC4Robot-Tool-FlattenOccs', 
                                                   'Flatten Occurrences', 
                                                   'Move bodies in child occs into the selected occurrence')
        
        # Connect to the command created event.
        sampleCommandCreated = FlattenOccCommandCreatedEventHandler()
        buttonSample.commandCreated.add(sampleCommandCreated)
        handlers.append(sampleCommandCreated)
        
        # Execute the command.
        buttonSample.execute()
        
        # Keep the script running.
        adsk.autoTerminate(False)
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


# Event handler for the commandCreated event.
class FlattenOccCommandCreatedEventHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()
        self.app = adsk.core.Application.get()
        self.des = adsk.fusion.Design.cast(self.app.activeProduct)
        self.ui = self.app.userInterface

    def notify(self, args):
        eventArgs = adsk.core.CommandCreatedEventArgs.cast(args)
        cmd = eventArgs.command

        # Create a command input
        inputs = cmd.commandInputs

        # Create a SelectionCommandInput, which can only select 1 occurrence
        selectInput: adsk.core.SelectionCommandInput = inputs.addSelectionInput(
            'selected-1',
            'Target Occurrence',
            'Selecte one occurrence as input occurrence.'
        )
        selectInput.addSelectionFilter("Occurrences")
        selectInput.setSelectionLimits(minimum=1, maximum=1)

        # Connect to the execute event.
        onExecute = FlattenOccCommandExecuteHandler()
        cmd.execute.add(onExecute)
        handlers.append(onExecute)


# Event handler for the execute event.
class FlattenOccCommandExecuteHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
        self.app = adsk.core.Application.get()
        self.des = adsk.fusion.Design.cast(self.app.activeProduct)
        self.ui = self.app.userInterface
        # self.textPalatte: adsk.core.TextCommandPalette = self.ui.palettes.itemById("TextCommands")
        # if not self.textPalatte.isVisible:
        #     self.textPalatte.isVisible = True
        # self.textPalatte.writeText("Start Command Execution")

    def flatten_occ(self, targetOcc: adsk.fusion.Occurrence):
        """
        Cut all sub occurrences' body into the input occurrence
        """
        # get list of all sub occurrences
        allSubOcc: List[adsk.fusion.Occurrence] = [] # a list to store the occs waiting for operations
        def dfs(node: adsk.fusion.Occurrence):
            allSubOcc.append(node)
            if node.childOccurrences.count == 0:
                return
            for child in node.childOccurrences:
                dfs(child)
        
        dfs(targetOcc)
        
        allSubOcc = allSubOcc[1:] # remove targetOcc

        # move all bodies in child occs into target occ
        for occ in allSubOcc:
            bodyList: adsk.fusion.BRepBodies = occ.bRepBodies
            for body in bodyList:
                body.moveToComponent(targetOcc)

        # remove occ that has no childOcc and bodies
        allSubOcc.reverse()
        for occ in allSubOcc:
            if occ.childOccurrences.count == 0 and occ.bRepBodies.count == 0:
                occ.deleteMe()

    def notify(self, args):
        eventArgs = adsk.core.CommandEventArgs.cast(args)

        # Code to react to the event.

        # Get the values from the command inputs
        inputs = eventArgs.command.commandInputs
        selectInput: adsk.core.SelectionCommandInput = inputs.itemById('selected-1')
        selection: adsk.core.Selection = selectInput.selection(0)
        entity: adsk.fusion.Occurrence = selection.entity
        
        try:

            # if self.des:
            self.flatten_occ(entity)
            message = self.app.userInterface.messageBox(
                "Finished flatten occurrence",
                "ACDC4Robot Tool"
            )  
        except:
            if self.ui:
                self.ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

        # Force the termination of the command.
        adsk.terminate()

           
def stop(context):
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        
        # Delete the command definition.
        cmdDef = ui.commandDefinitions.itemById('ACDC4Robot-Tool-FlattenOccs')
        if cmdDef:
            cmdDef.deleteMe()            
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))