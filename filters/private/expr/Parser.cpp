
#include "Parser.hpp"

namespace pdal
{
namespace expr
{

bool Parser::parse(const std::string& s)
{
    m_lexer.lex(s);
    return expression();
}


void Parser::prepare(PointLayoutPtr l)
{
    if (m_nodes.size() != 1)
    {
        std::cerr << "Can't prepare.  Node tree not properly parsed.\n";
        return;
    }
    m_nodes.top()->prepare(l);
}


double Parser::eval(PointRef& p) const
{
    if (m_nodes.size() != 1)
    {
        std::cerr << "Can't evaluate.  Node tree not properly parsed.\n";
        return 0;
    }
    return m_nodes.top()->eval(p);
}


// ABELL - This needs reworking since we may pop as another class.
Token Parser::popToken(TokenClass cls)
{
    if (m_tokens.empty())
        return m_lexer.get(cls);
    Token tok = m_tokens.top();
    m_tokens.pop();
    return tok;
}

void Parser::pushToken(const Token& tok)
{
    m_tokens.push(tok);
}


bool Parser::expression()
{
    if (!addexpr())
        return false;
    return true;
}

bool Parser::addexpr()
{
    if (!multexpr())
        return false;

    while (true)
    {
        Token tok = popToken(TokenClass::Operator);
        NodeType type;
        if (tok.type() == TokenType::Plus)
            type = NodeType::Plus;
        else if (tok.type() == TokenType::Minus)
            type = NodeType::Minus;
        else
        {
            pushToken(tok);
            return true;
        }

        if (!multexpr())
        {
            m_error = "Operator not followed by rvalue.";
            return false;
        }

        NodePtr right(popNode());
        NodePtr left(popNode());

        ValNode *leftVal = dynamic_cast<ValNode *>(left.get());
        ValNode *rightVal = dynamic_cast<ValNode *>(right.get());
        if (leftVal && rightVal)
        {
            double v =
                (type == NodeType::Plus) ?
                leftVal->value() + rightVal->value() :
                leftVal->value() - rightVal->value();
            pushNode(NodePtr(new ValNode(v)));
        }
        else
            pushNode(NodePtr(new BinNode(type, std::move(left),
                std::move(right))));
    }
    return true;
}

bool Parser::multexpr()
{
    if (!primary())
        return false;

    while (true)
    {
        Token tok = popToken(TokenClass::Operator);
        NodeType type;
        if (tok.type() == TokenType::Multiply)
            type = NodeType::Multiply;
        else if (tok.type() == TokenType::Divide)
            type = NodeType::Divide;
        else
        {
            pushToken(tok);
            return true;
        }

        if (!primary())
        {
            m_error = "Operator not followed by rvalue.";
            return false;
        }

        NodePtr right = popNode();
        NodePtr left = popNode();

        ValNode *leftVal = dynamic_cast<ValNode *>(left.get());
        ValNode *rightVal = dynamic_cast<ValNode *>(right.get());
        if (leftVal && rightVal)
        {
            double v;
            if (type == NodeType::Multiply)
                v = leftVal->value() * rightVal->value();
            else
            {
                if (rightVal->value() == 0.0)
                {
                    m_error = "Divide by 0.";
                    return false;
                }
                v = leftVal->value() / rightVal->value();
            }
            pushNode(NodePtr(new ValNode(v)));
        }
        else
            pushNode(NodePtr(new BinNode(type, std::move(left),
                std::move(right))));
    }
    return true;
}


bool Parser::primary()
{
    Token tok = popToken();
    if (tok.type() == TokenType::Number)
    {
        pushNode(NodePtr(new ValNode(tok.dval())));
        return true;
    }
    else if (tok.type() == TokenType::Dimension)
    {
        pushNode(NodePtr(new VarNode(tok.sval())));
        return true;
    }
    else if (tok.type() == TokenType::Eof)
        return true;

    pushToken(tok);
    return parexpr();
}


bool Parser::parexpr()
{
    Token tok = popToken();
    if (tok.type() != TokenType::Lparen)
    {
        pushToken(tok);
        return false;
    }

    if (!expression())
        return false;

    tok = popToken();
    if (tok.type() != TokenType::Rparen)
    {
        pushToken(tok);
        return false;
    }
    return true;
}

} // namespace expr
} // namespace pdal
