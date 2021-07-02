#ifndef MATH_HPP
#define MATH_HPP


#include <SFML/Graphics.hpp>


float bound(float min, float max, float val);
float dot_product(sf::Vector2f A, sf::Vector2f B);
float length_squared(sf::Vector2f A);
float length(sf::Vector2f A);
sf::Vector2f normalize(sf::Vector2f A);
float cross_product(sf::Vector2f A, sf::Vector2f B);


#endif