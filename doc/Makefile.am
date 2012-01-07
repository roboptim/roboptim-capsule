# 
#  Copyright 2007 CNRS
#
#  Author: Florent Lamiraux
# 


hppModeldir = $(datadir)/doc/hppModel/html

EXTRA_DIST = \
	header.html \
	footer.html \
	package.css \
	pictures/footer.jpg \
	figures/classDiagramRobot.dot \
	figures/classDiagramRobot.png \
	figures/classDiagramJoint.dot \
	figures/classDiagramJoint.png \
	figures/classDiagramHumanoid.dot \
	figures/classDiagramHumanoid.png \
	figures/classDiagramBody.dot \
	figures/classDiagramBody.png \
	additionalDoc/package.h 


install: html-doc 
	mkdir -p $(hppModeldir)
	$(INSTALL_DATA) html/* $(hppModeldir)
	mkdir -p $(hppModeldir)/pictures
	$(INSTALL_DATA) $(srcdir)/pictures/* $(hppModeldir)/pictures
	$(INSTALL_DATA) hppModel.doxytag $(hppModeldir)

html-doc: package.dox
	doxygen $<
