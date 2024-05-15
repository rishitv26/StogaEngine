#include "stogaEngine/utilities.h"
#include "stogaEngine/data.h"

const std::string& engine::Component::getID() const
{
	return ID;
}

void engine::Component::setID(const std::string& id)
{
	ID = id;
}

bool engine::Component::isActive()
{
	return isActive;
}

void engine::Component::run(int analog, int analog2)
{
	run(analog);
}

void engine::Component::run(int analog, int analog2, int analog3)
{
	run(analog);
}

void engine::Component::brake()
{
	run(0);
}

engine::Data<double> engine::Component::getData()
{
	return Data<double>();
}

void engine::Utilities::checkIfIDExists(const std::string& ID)
{
	for (int i = 0; i < components.size(); ++i) {
		if (components[i].getID() == ID) {
			// throw an exception
			// TODO
		}
	}
}

engine::Utilities::Utilities(std::initializer_list<Component> components)
{
	addComponents(components);
}

void engine::Utilities::addComponent(Component& component)
{
	checkIfIDExists(component.getID());
	components.push_back(component);
}

void engine::Utilities::addComponents(std::initializer_list<Component> comps)
{
	for (auto& i : comps) {
		checkIfIDExists(i.getID());
		components.push_back(i);
	}
}

void engine::Utilities::removeComponent(const std::string& key)
{
	for (int i = 0; i < components.size(); ++i) {
		if (components[i].getID() == key) {
			components.erase(components.begin() + i);
			return;
		}
	}
}

engine::Component& engine::Utilities::getComponentByID(const std::string& ID)
{
	for (int i = 0; i < components.size(); ++i) {
		if (components[i].getID() == ID) {
			return components[i];
		}
	}
	// todo
	// throw an exception here.
	return;
}

void engine::Utilities::brakeAll()
{
	for (int i = 0; i < components.size(); ++i) {
		components[i].brake();
	}
}

std::vector<engine::Component>& engine::Utilities::getVectorFormat()
{
	return components;
}
