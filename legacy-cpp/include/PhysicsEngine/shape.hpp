#ifndef SHAPE_HPP
#define SHAPE_HPP


struct Shape
{

};


struct Circle : public Shape
{
    float radius;
};


struct Polygon : public Shape
{
    sf::Vector2f min;
    sf::Vector2f max;
};


#endif