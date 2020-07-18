# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

#/***************************************************************************/
#/* Raspberry Pi ELM327 OBBII CAN BUS Diagnostic Software.                  */
#/*                                                                         */
#/* (C) Jason Birch 2018-05-15 V1.04                                        */
#/*                                                                         */
#/* Class: Confirm                                                          */
#/* Display a dialog showing a YES/NO question and get the response.        */
#/***************************************************************************/



import pygame
import Visual
import Button



class Confirm(Visual.Visual):
	def __init__(self, ThisSurface, Name, Text, xLen = 0, TickOnly = False):
		# Confirm covers full display surface to prevent other interface items being clicked before the confirmation has been clicked.
		Visual.Visual.__init__(self, ThisSurface, Name, Visual.PRESS_NONE, 0, 0, Button.Visual.BUTTON_HEIGHT, Button.Visual.BUTTON_HEIGHT, Text)

		# Confirm covers full display, but only draw in middle so some interface can still be seen, but not used.
		if xLen == 0:
			self.xLen = self.DisplayXLen / 3
		else:
			self.xLen = xLen
		self.yLen = self.DisplayYLen / 2.5
		self.xPos = (self.DisplayXLen - self.xLen) / 2
		self.yPos = (self.DisplayYLen - self.yLen) / 2
		self.ColourFill = self.ColourDialog

		# Buttons displayed on the confirm dialog.
		self.Buttons = {
			# Confirm covers full display with this button to prevent other user interface being used while dialog visible.
			"UI_BLOCKER" : Button.Button(self.ThisSurface, "UI_BLOCKER", Visual.PRESS_NONE, -self.xLen, -self.yLen, self.DisplayXLen, self.DisplayYLen, "UI_BLOCKER"),
		}
		if TickOnly == False:
			self.Buttons["YES"] = Button.Button(self.ThisSurface, "YES", Visual.PRESS_DOWN, Button.Visual.BUTTON_HEIGHT, self.yLen - 1.5*Button.Visual.BUTTON_HEIGHT, Button.Visual.BUTTON_HEIGHT, Button.Visual.BUTTON_HEIGHT, "IMAGE:ICONS/Yes.png")
			self.Buttons["NO"] = Button.Button(self.ThisSurface, "NO", Visual.PRESS_DOWN, self.xLen - 2*Button.Visual.BUTTON_HEIGHT, self.yLen - 1.5*Button.Visual.BUTTON_HEIGHT, Button.Visual.BUTTON_HEIGHT, Button.Visual.BUTTON_HEIGHT, "IMAGE:ICONS/No.png")
		else:
			self.Buttons["YES"] = Button.Button(self.ThisSurface, "YES", Visual.PRESS_DOWN, (self.xLen - Button.Visual.BUTTON_HEIGHT) / 2, self.yLen - 1.5*Button.Visual.BUTTON_HEIGHT, Button.Visual.BUTTON_HEIGHT, Button.Visual.BUTTON_HEIGHT, "IMAGE:ICONS/Yes.png")

		self.Buttons["UI_BLOCKER"].SetVisible(False)



#/************************************************************************/
#/* Check if an event occurred in this dialog area. Perform any required */
#/* actions and let the caller know if the dialog was touched.           */
#/************************************************************************/
	def IsEvent(self, EventType, xPos, yPos, PointerButton, xOffset = 0, yOffset = 0):
		Result = Visual.Visual.IsEvent(self, EventType, xPos, yPos, PointerButton, xOffset, yOffset)

		# Handle dialog events.
		if Result != False:
			# Tell caller the name of the gague that was touched.
			Result["GADGIT"] = self.Name
			# Tell caller the name of the button on the gague that was touched.
			Result["BUTTON"] = ""
			for ThisButton in self.Buttons:
				ButtonResult = self.Buttons[ThisButton].IsEvent(EventType, xPos, yPos, PointerButton, self.xPos, self.yPos)
				if ButtonResult != False and self.Buttons[ThisButton].GetName() != "UI_BLOCKER":
					Result["BUTTON"] = ThisButton
					break
		else:
			# Always return true, no other user interface is available until this dialog answered.
			Result = {}
			Result["GADGIT"] = self.Name
			Result["BUTTON"] = ""

		return Result



#/*********************************************/
#/* Draw this dialog on the provided surface. */
#/*********************************************/
	def Display(self, ThisSurface, xOffset = 0, yOffset = 0):
		# Display dialog background and border.
		pygame.draw.rect(ThisSurface, self.ColourFill, (xOffset + self.xPos, yOffset + self.yPos, self.xLen, self.yLen), 0)
		pygame.draw.rect(ThisSurface, self.ColourBorder, (xOffset + self.xPos, yOffset + self.yPos, self.xLen, self.yLen), 4)

		# Display all buttons on the gadgit.
		for ThisButton in self.Buttons:
			self.Buttons[ThisButton].Display(self.ThisSurface, self.xPos, self.yPos)

		# Draw any super class elements.
		Visual.Visual.Display(self, ThisSurface, xOffset, -(self.yLen/5))

