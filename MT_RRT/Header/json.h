/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/
 
#ifndef _MT_RRT_JSON_H__
#define _MT_RRT_JSON_H__

#include <vector>
#include <string>

namespace MT_RTT
{		
	/** \brief This class can be used to decode or encode a simple category of json structures. 
	\details It is not possible to handle a generic json. Indeed, the structure to parse must be an array of node, each having a name and a matrix of values.
	The information describing each node are contained in json_parser::field.
	*/
	class json_parser {
	public:
		/** \brief The structure describing each node in the array to encode
		*/
		struct field {
			std::string							name;
			std::vector<std::vector<float>>		values;
		};

///////////////////////////////////
// 			decoding 			 //
///////////////////////////////////

		/** \brief Performs decoding from an input string. 
		* @param[in] JSON the string to decode
		* @param[out] return the parsed array of nodes.
		*/
		static std::vector<field> parse_JSON(const std::string& JSON);

		/** \brief Read the json contained in a specified file. The returned result can be later parsed using parse_JSON.
		* @param[in] file_location the file to read
		* @param[out] return the content of the file to read.
		*/
		static std::string		  get_JSON_from_file(const std::string& file_location);
		
		/** \brief Get a specific field with a specific name.
		\details In case the specified field does not exists, NULL is returned.
		* @param[in] fields the array of nodes that contain the field to search
		* @param[in] name_to_search the name of the node to return
		* @param[out] return the values associated to the node having the specified name.
		*/
		static const std::vector<std::vector<float>>* get_field(const std::vector<field>& fields, const std::string& name_to_search);


///////////////////////////////////
// 			encoding 			 //
///////////////////////////////////

		/** \brief Performs encoding. 
		* @param[in] fields the data to encode
		* @param[out] return the encoded json as a string.
		*/
		static std::string		  load_JSON(const std::vector<field>& fields);

		/** \brief Performs encoding of a single numerical array.
		* @param[in] buffer the buffer of numbers to encode
		* @param[in] Size the size of the buffer of numbers to encode
		* @param[out] return the encoded json.
		*/
		static std::string		  load_JSON(const float* buffer, const size_t& Size);
		
		/** \brief Performs encoding of a single matrix of numbers (each row can have a different size).
		* @param[in] values the matrix of numbers to encode
		* @param[out] return the encoded json.
		*/
		static std::string		  load_JSON(const std::vector<std::vector<float>>& values);
		
		/** \brief Similar to json_parser::load_JSON(const std::vector<std::vector<float>>& values), but passing
		a single buffer, whose row subdivision is expressed by Sizes.
		* @param[in] buffer the matrix of numbers to encode
		* @param[in] Sizes the size of each row
		* @param[out] return the encoded json.
		*/
		static std::string		  load_JSON(const float* buffer, const std::vector<size_t>& Sizes);
	};
}

#endif