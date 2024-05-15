/**
 * @file utilities.h
 * @author Rishit Varshney
 * @brief This file contains the utilities and components classes that compose other parts of the robots.
 * @version 0.1
 * @date 2024-05-14
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef UTILITIES_SE_H
#define UTILITIES_SE_H

#include <string>
#include <vector>
#include "data.h"

namespace engine {
	class Component {
	protected:
		std::string ID;
		bool isActive;
	public:
		Component() {};
		/*
		* Get the ID of this component.
		* Used to distinctify this object from the others.
		* 
		* @returns the ID representation as a std::string.
		*/
		const std::string& getID() const;
		/*
		* Set the ID of this component.
		* Used to distinctify this object from the others.
		* 
		* @param ID the new ID
		*/
		void setID(const std::string& ID);
		/*
		* Activate the component.
		* This may vary for each component, and enables the run command to do its job.
		* May do preliminary actions like activating PTO's or changing ports.
		*/
		virtual void activate() = 0;
		/*
		* Deactivate the component.
		* This may vary for each component, and enables the run command to disable.
		* 
		* should call brake() before activation.
		* May do preliminary actions like deactivating PTO's or changing ports.
		*/
		virtual void deactivate() = 0;
		/*
		* Returns if the status is active or not.
		* true means its activated, false means its not.
		* 
		* @returns the status.
		*/
		virtual bool isActive();
		/*
		* Runs the component, given some analog input.
		* If component does not need analog, the parameter will not affect the actions.
		* 
		* @param analog input for action.
		*/
		virtual void run(int analog) = 0;
		/*
		* Runs the component, given some analog input.
		* If component does not need analog, the parameter will not affect the actions.
		*
		* @param analog input for action.
		* @param analog2 input for action.
		*/
		virtual void run(int analog, int analog2);
		/*
		* Runs the component, given some analog input.
		* If component does not need analog, the parameter will not affect the actions.
		*
		* @param analog input for action.
		* @param analog2 input for action.
		* @param analog3 input for action.
		*/
		virtual void run(int analog, int analog2, int analog3);
		/*
		* Stops this component immediatly.
		* Does not deactivate though.
		*/
		virtual void brake();
		/*
		* Gets data from this component.
		* This data may be in any format depending on whats best.
		* Data used here is in terms of a double.
		* 
		* @returns data in terms of doubles.
		*/
		virtual Data<double> getData();
	};

	class Utilities {
	protected:
		std::vector<Component> components;
		/*
		* Checks if this ID exists. If it does, throws an exception. 
		*/
		void checkIfIDExists(const std::string& ID);
	public:
		Utilities() {};
		/*
		* Constructs a Utility object via a list of components
		* Will throw an exception if the component with same ID already exists.
		* 
		* @param components the list of components to add.
		*/
		Utilities(std::initializer_list<Component> components);
		/*
		* Adds a component to this Utilities object.
		* Will throw an exception if the component with same ID already exists.
		* 
		* @param component the component wanted to be added.
		*/
		void addComponent(Component& component);
		/*
		* Adds components to this Utilities object.
		* Will throw an exception if the components with same IDs already exists.
		*
		* @param components the components wanted to be added.
		*/
		void addComponents(std::initializer_list<Component> components);
		/*
		* Removes a component by key.
		* Will do nothing if key does not exist.
		* 
		* @param key the key to remove component by.
		*/
		void removeComponent(const std::string& key);
		/*
		* Get a component based by key.
		* Will return an exception if key does not exist.
		* 
		* @param ID the ID to return
		* @returns the Component from the ID
		*/
		Component& getComponentByID(const std::string& ID);
		/*
		* Will break all Components.
		* Usefull for disable()
		*/
		void brakeAll();
		/*
		* Gets a std::vector representation of this Utilities object.
		* 
		* @returns std::vector representation.
		*/
		std::vector<Component>& getVectorFormat();
	};
}

#endif // !UTILITIES_SE_H
