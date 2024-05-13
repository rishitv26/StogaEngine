/**
 * @file data.h
 * @author Rishit Varshney
 * @brief This file contains all items that store and organize data from sensors.
 * @version 0.1
 * @date 2024-05-13
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef DATA_SE_H
#define DATA_SE_H

#include <map>
#include <string>

namespace engine {
	/*
	* Represents data as a compact class for neater organization and abstraction.
	*/
	template <typename T>
	class Data {
	private:
	public:
		/*
		* Gets the ID of this data object.
		* neccessay to distiguish between multiple data objects.
		* 
		* @returns the ID as a std::string&
		*/
		const std::string& getID() const;
		/*
		* Sets the ID of this data object.
		* neccessay to distiguish between multiple data objects.
		* 
		* @param name the new ID
		*/
		void setID(char* name);
		/*
		* Adds a new set of data in succession.
		* 
		* @param key the key labeled to this new data.
		* @param data the data stored in this entry.
		*/
		void addData(std::string& key, T data);
		/*
		* Removes an entry of data with the given label.
		* 
		* @param key the key to remove.
		*/
		void removeData(std::string& key);
		/*
		* Returns some data with the given label.
		* Will throw an exception if the given label was not found.
		* 
		* @param key the key to find.
		* @returns the data associated with the key.
		*/
		T getData(std::string& key) const;
		/*
		* Returns all of the data as an std::map
		* 
		* @returns all data within this object.
		*/
		std::map<std::string, T>& all() const;
	};
}

#endif // !DATA_SE_H
