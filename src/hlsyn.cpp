// ECE 474A/574A - Example C++ program with support for checking commandline
//                 arguments required for Assignment 4
// Author: Charles Leichner
// Date: 12/5/12

#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <sstream>
#include <vector>
#include <climits>
using namespace std;

typedef unsigned int state;
bool DEBUG = false;
state current_state = 0;
state unique_state() {
    return current_state++;
}

state current_end_state = UINT_MAX;
state unique_end_state() {
    return current_end_state--;
}

void true_or_die(bool constraint, string error_message) {
    if (!constraint) {
        cerr << "Error: " << error_message << '\n';
        exit(-1);
    }
}

bool is_not_alphanumeric(char c) {
    return !(isalpha(c) || isdigit(c));
}

bool all_alphanumeric(string str) {
    return find_if(str.begin(), str.end(), is_not_alphanumeric) == str.end();
}

string alphanumeric(string str) {
    true_or_die(all_alphanumeric(str),
            "identifiers must consist of letters and numbers");
    return str;
}

long unique() {
    static long n = 0;
    return n++;
}

typedef enum {
    EMPTY,
    VAR,
    DO,
    WHILE,
    IF,
    ASSIGN,
    PLUS,
    MINUS,
    MULT,
    LT,
    GT,
    EQ,
    QMARK,
    COLON,
    L_SHIFT,
    R_SHIFT,
    INPUTS,
    OUTPUTS,
    REGS,
    L_CURLY,
    R_CURLY,
    L_PAREN,
    R_PAREN,
    NEWLINE,
    EOF_TOKEN,
} TokenType;

class Token {
   friend ostream& operator<<(ostream& output, const Token& t);
  public:
    TokenType type;
    string contents;

    Token() : type(EMPTY), contents() {}
    Token(TokenType type, string contents) : type(type), contents(contents) {}

    static Token newline() {
        return Token(NEWLINE, "\n");
    }

    static Token eof() {
        return Token(EOF_TOKEN, "eof");
    }

    static Token from(string s) {
        if (s == ")") {
            return Token(R_PAREN, s);
        } else if (s == "(") {
            return Token(L_PAREN, s);
        } else if (s == "}") {
            return Token(R_CURLY, s);
        } else if (s == "{") {
            return Token(L_CURLY, s);
        } else if (s == "INPUTS") {
            return Token(INPUTS, s);
        } else if (s == "OUTPUTS") {
            return Token(OUTPUTS, s);
        } else if (s == "REGS") {
            return Token(REGS, s);
        } else if (s == ">>") {
            return Token(R_SHIFT, s);
        } else if (s == "<<") {
            return Token(L_SHIFT, s);
        } else if (s == ":") {
            return Token(COLON, s);
        } else if (s == "?") {
            return Token(QMARK, s);
        } else if (s == "==") {
            return Token(EQ, s);
        } else if (s == ">") {
            return Token(GT, s);
        } else if (s == "<") {
            return Token(LT, s);
        } else if (s == "*") {
            return Token(MULT, s);
        } else if (s == "-") {
            return Token(MINUS, s);
        } else if (s == "+") {
            return Token(PLUS, s);
        } else if (s == "=") {
            return Token(ASSIGN, s);
        } else if (s == "if") {
            return Token(IF, s);
        } else if (s == "while") {
            return Token(WHILE, s);
        } else if (s == "do") {
            return Token(DO, s);
        } else {
            true_or_die(all_alphanumeric(s),
                "variables must consist only of alphanumeric characters");
            return Token(VAR, s);
        }
    }

    bool is_op() {
        return this->type == PLUS ||
            this->type == MINUS ||
            this->type == MULT ||
            this->type == LT ||
            this->type == GT ||
            this->type == EQ ||
            this->type == QMARK ||
            this->type == COLON ||
            this->type == L_SHIFT ||
            this->type == R_SHIFT;
    }

};

std::ostream& operator<< (std::ostream& os, const Token& t) {
    switch (t.type) {
        case VAR: os << "VAR"; break;
        case DO: os << "DO"; break;
        case WHILE: os << "WHILE"; break;
        case IF: os << "IF"; break;
        case ASSIGN: os << "ASSIGN"; break;
        case PLUS: os << "PLUS"; break;
        case MINUS: os << "MINUS"; break;
        case MULT: os << "MULT"; break;
        case LT: os << "LT"; break;
        case GT: os << "GT"; break;
        case EQ: os << "EQ"; break;
        case QMARK: os << "QMARK"; break;
        case COLON: os << "COLON"; break;
        case L_SHIFT: os << "L_SHIFT"; break;
        case R_SHIFT: os << "R_SHIFT"; break;
        case INPUTS: os << "INPUTS"; break;
        case OUTPUTS: os << "OUTPUTS"; break;
        case REGS: os << "REGS"; break;
        case L_CURLY: os << "L_CURLY"; break;
        case R_CURLY: os << "R_CURLY"; break;
        case L_PAREN: os << "L_PAREN"; break;
        case R_PAREN: os << "R_PAREN"; break;
        case EMPTY: os << "EMPTY"; break;
        case NEWLINE: os << "NEWLINE"; break;
        case EOF_TOKEN: os << "EOF_TOKEN"; break;
        default:
            true_or_die(false, "unknown token!");
    }
    return os;
}

class TokenStream {
  public:
    TokenStream(istream& in) : in(in), current(Token(EOF_TOKEN, "EOF")),
                               c_line(NULL), word(split.end()) {
        ++(*this);
    }

    Token& operator*() {
        return current;
    }

    Token* operator->() {
        return &current;
    }

    TokenStream& operator ++(int) {
        return ++(*this);
    }

    // wraps Token::from to ignore comments
    Token next_token() {
        if (*word == "//") {
            word = split.end();
            ++(*this);
        }
        if (DEBUG) {
            cout << Token::from(*word) << " -> " << *word << '\n';
        }
        return Token::from(*word);
    }

    TokenStream& operator ++() {
        if (word != split.end()) {
            ++word;
        }
        if (word != split.end()) {
            current = next_token();
        } else if (getline(in, line)) {
            delete c_line;
            c_line = new stringstream(line);
            line_start = istream_iterator<string>(*c_line);
            split = vector<string>(line_start, line_end);
            word = split.begin();
            if (word != split.end()) {
                current = next_token();
            } else {
                if (DEBUG) {
                    cout << "NEWLINE\n";
                }
                current = Token::newline();
            }
        } else {
            if (DEBUG) {
                cout << "EOF\n";
            }
            current = Token::eof();
        }
        return *this;
    }

    ~TokenStream() {
        delete c_line;
    }

  private:

    // Disallow copy and assign
    TokenStream(const TokenStream&);
    void operator=(const TokenStream&);

    istream& in;
    Token current;

    // used for splitting on/removing whitespace
    string line;
    stringstream* c_line;
    istream_iterator<string> line_start;
    istream_iterator<string> line_end;
    vector<string> split;
    vector<string>::iterator word;
};

typedef enum {
    NONE,
    INPUT,
    OUTPUT,
    REG,
} SymbolType;

class Symbol {
    friend std::ostream& operator<< (std::ostream& os, const Symbol& s);
  public:
    string name;
    SymbolType type;

    Symbol(string name, SymbolType t) : name(name), type(t) {}
    Symbol() : name("NONE"), type(NONE) {}

    Symbol(const Symbol& that) : name(that.name), type(that.type) {}

    Symbol& operator=(const Symbol& other) {
        this->name = other.name;
        this->type = other.type;
        return *this;
    }
};

std::ostream& operator<< (std::ostream& os, const Symbol& s) {
    switch (s.type) {
        case INPUT: os << "INPUT"; break;
        case OUTPUT: os << "OUTPUT"; break;
        case REG: os << "REG"; break;
        default:
            true_or_die(false, "unknown symbol type");
    }
    return os;
}

class ExpressionBlock;

// Base policy for scheduling blocks of arithmatic expressions
class Schedule {
  public:
    virtual vector<ExpressionBlock> operator()(ExpressionBlock) = 0;
    virtual ~Schedule() {};
};

class Statements {
  public:
    vector<Statements*> contents;

    Statements() {}
    Statements(const Statements& that) : contents(that.contents) {}
    virtual ~Statements() {
        for (vector<Statements*>::iterator it = contents.begin();
            it != contents.end(); ++it) {
            delete *it;
        }
    }

    virtual void generate_fsm(ostream& os, Schedule& schedule,
                              state start,  state end) {
        state inner_start = start;
        state inner_end;

        if (!contents.empty()) {
            for (vector<Statements*>::iterator it = contents.begin();
                it != contents.end(); ++it) {
                if (it+1 == contents.end()) {
                    inner_end = end;
                } else {
                    inner_end = unique_state();
                }
                (*it)->generate_fsm(os, schedule, inner_start, inner_end);
                inner_start = inner_end;
            }
        } else {
            os << "        " << start << ": begin\n";
            os << "            State <= " << end << ";\n";
            os << "        end\n";
        }
    }

    void append(Statements* s) {
        contents.push_back(s);
    }
};

// individual expression
class Expression {
    friend std::ostream& operator<< (std::ostream& os, const Expression& s);
  public:
    TokenType op;
    Symbol result;
    vector<Symbol> arguments;

    Expression() : op(EMPTY) {}
    Expression(const Expression& that) :
        op(that.op), result(that.result), arguments(that.arguments) {}

    void add_argument(Symbol s) {
        arguments.push_back(s);
    }
};

std::ostream& operator<< (std::ostream& os, const Expression& s) {
    os << s.result.name << " ";
    switch (s.op) {
        case ASSIGN: os << "="; break;
        case PLUS: os << "+"; break;
        case MINUS: os << "-"; break;
        case MULT: os << "*"; break;
        case LT: os << "<"; break;
        case GT: os << ">"; break;
        case EQ: os << "=="; break;
        case QMARK: os << "?"; break;
        case L_SHIFT: os << "<<"; break;
        case R_SHIFT: os << ">>"; break;
        default:
            true_or_die(false, "unknown operator");
    }

    for (vector<Symbol>::const_iterator arg = s.arguments.begin();
         arg != s.arguments.end(); ++arg) {
        os << " " << arg->name;
    }

    return os;
}

// Schedulable block of contiguous expressions
class ExpressionBlock : public Statements {
  public:
    vector<Expression> expressions;
    ExpressionBlock() : Statements() {}

    ExpressionBlock(const ExpressionBlock& that) :
        Statements(), expressions(that.expressions) {}

    virtual Statements* statements() {
        return this;
    }
    void add_expression(Expression e) {
        expressions.push_back(e);
    }
    vector<Expression>::iterator begin() {
        return expressions.begin();
    }
    vector<Expression>::iterator end() {
        return expressions.end();
    }
    virtual void generate_fsm(ostream& os, Schedule& schedule,
                              state start,  state end) {
        state inner_start = start;
        state inner_end;

        vector<ExpressionBlock> scheduled_blocks = schedule(*this);
        for (vector<ExpressionBlock>::iterator block = scheduled_blocks.begin();
            block != scheduled_blocks.end(); ++block) {
            os << "        " << inner_start << ": begin\n";
            for (vector<Expression>::iterator contents = block->begin();
                 contents != block->end(); ++contents) {
                os << "            " << contents->result.name << " <= ";
                vector<Symbol>::iterator arguments = contents->arguments.begin();
                os << arguments->name;
                if (arguments != contents->arguments.end()) {
                    ++arguments;
                    if (arguments != contents->arguments.end()) {
                        os << " ";
                        switch (contents->op) {
                            case PLUS:
                                os << "+";
                                break;
                            case MINUS:
                                os << "-";
                                break;
                            case MULT:
                                os << "*";
                                break;
                            case LT:
                                os << "<";
                                break;
                            case GT:
                                os << ">";
                                break;
                            case EQ:
                                os << "==";
                                break;
                            case QMARK:
                                os << "?";
                                break;
                            case L_SHIFT:
                                os << "<<";
                                break;
                            case R_SHIFT:
                                os << ">>";
                                break;
                            default:
                                true_or_die(false,
                                     "unknown operator in generate_fsm");
                        }
                        os << " " << arguments->name;
                        ++arguments;
                        if (arguments != contents->arguments.end()) {
                            os << " : " <<  arguments->name;
                        }
                    }
                }
                os << ";\n";
                if (block+1 == scheduled_blocks.end()) {
                    inner_end = end;
                } else {
                    inner_end = unique_state();
                }
                os << "            State <= "<< inner_end <<";\n";
                os << "        end\n";
                inner_start = inner_end;
            }
        }
    }
};

class While : public Statements {
  public:
      Symbol condition;
      // contents of While is Statements::statements
};

class Do : public Statements {
  public:
      Symbol condition;
      // contents of Do is Statements::statements
};

class If : public Statements {
  public:
      Symbol condition;
      // contents of If is Statements::statements

    virtual void generate_fsm(ostream& os, Schedule& schedule, state start, state end) {
        state inner_start = unique_state();
        state inner_end = unique_end_state();

        os << "\n        // IF STATEMENT\n";
        os << "        " << start << ": begin\n";
        os << "            if (" << condition.name << ") begin\n";
        os << "                State <= " << inner_start << ";\n";
        os << "            end else begin\n";
        os << "                State <= " << inner_end << ";\n";
        os << "            end\n";
        os << "        end\n";

        Statements::generate_fsm(os, schedule, inner_start, inner_end);

        os << "        " << inner_end << ": begin\n";
        os << "            State <= " << end << ";\n";
        os << "        end // END IF STATEMENT\n\n";
    }
};

class NoSchedule : public Schedule {
  public:
    virtual vector<ExpressionBlock> operator()(ExpressionBlock e) {
        vector<ExpressionBlock> scheduled_blocks;
        for (vector<Expression>::iterator it = e.begin();
            it != e.end(); ++it) {
            ExpressionBlock block;
            block.add_expression(*it);
            scheduled_blocks.push_back(block);
        }
        return scheduled_blocks;
    }
};

typedef map<string, Symbol> SymbolTable;

class Program {
public:
    Program(SymbolTable symbols, Statements* statements) :
        symbols(symbols),
        statements(statements) {}

    void generate_fsm(ostream& os, Schedule& schedule) {
        os << "`timescale 1ns / 1ps\n";
        os << "module HLSM(Clk, Rst, Start, Done";
        for (SymbolTable::iterator sym = symbols.begin();
             sym != symbols.end(); ++sym) {
            if (sym->second.type == INPUT || sym->second.type == OUTPUT) {
                os << ", " << sym->first;
            }
        }
        os << ");\n";
        os << "input Clk, Rst, Start;\n";
        os << "output reg Done;\n";
        os << "reg[31:0] State;\n";
        for (SymbolTable::iterator sym = symbols.begin();
             sym != symbols.end(); ++sym) {
            if (sym->second.type == INPUT) {
                os << "input [31:0] " << sym->first << ";\n";
            } else if (sym->second.type == OUTPUT) {
                os << "output reg [31:0] " << sym->first << ";\n";
            } else if (sym->second.type == REG) {
                os << "reg [31:0] " << sym->first << ";\n";
            } else {
                os << "// NONE -> " << sym->first << ";\n";
            }
        }
        os << "\nalways @(posedge Clk) begin\n";
        os << "    if (Rst == 1) begin\n";
        os << "        State <= 0;\n";
                       // zero ouputs;
        os << "    end else begin\n";
        os << "        case (State)\n";
        os << "        " << unique_state() << ": begin\n";
        os << "            if (Start) begin\n";
        os << "                State <= 1;\n";
        os << "                Done <= 0;\n";
        os << "            end else begin\n";
        os << "                State <= 0;\n";
        os << "            end\n";
        os << "        end\n";

        state end_state = unique_end_state();
        statements->generate_fsm(os, schedule, unique_state(), end_state);

        os << "        " << end_state << ": begin\n";
        os << "            Done <= 1;\n";
        os << "            State <= 0;\n";
        os << "        end\n";
        os << "        endcase\n";
        os << "    end\n";
        os << "end\n";
        os << "\nendmodule\n";
    }

    ~Program() {
        delete statements;
    }

    SymbolTable symbols;
    Statements* statements;
};

class Parser {
  public:
    Parser(istream& in) : t(in) {};

    Program program() {
        declarations();
        return Program(s, statements());
    }

  private:
    class SymtableInsertion {
      public:
        Parser* p;
        TokenType token_type;
        SymbolType symbol_type;
        SymtableInsertion(Parser* p, TokenType t, SymbolType s) :
            p(p), token_type(t), symbol_type(s) {}

        bool operator()(TokenStream& t, SymbolTable& s) {
            p->ignore_newlines();
            if (t->type == token_type) {
                t++;
            } else {
                return false;
            }

            p->accept_colon();

            while (t->type == VAR) {
                true_or_die(s.find(t->contents) == s.end(),
                        "attempting to redefine variable " + t-> contents);
                s[t->contents] = Symbol(t->contents, symbol_type);
                t++;
            }
            return true;
        }
    };

    void declarations() {
        // Declarations can come in any order, compensate with this by iterating
        // over a list of declarations we haven't seen yet.
        vector<SymtableInsertion> operations;
        operations.push_back(SymtableInsertion(this, INPUTS, INPUT));
        operations.push_back(SymtableInsertion(this, OUTPUTS, OUTPUT));
        operations.push_back(SymtableInsertion(this, REGS, REG));
        vector<SymtableInsertion> remaining;

        int num_ops = operations.size();
        for (int i = 0; i < num_ops; ++i) {
            for (vector<SymtableInsertion>::iterator op = operations.begin();
                op != operations.end(); ++op) {
                if (!(*op)(t, s)) {
                    remaining.push_back(*op);
                }
            }
            operations = remaining;
            remaining.clear();
        }
    }

    Statements* statements() {
        Statements* statements = new Statements();
        while (true) {
            ignore_newlines();
            if (t->type == VAR) {
                statements->append(expressions());
            } else if (t->type == WHILE) {
                statements->append(while_());
            } else if (t->type == DO) {
                statements->append(do_());
            } else if (t->type == IF) {
                statements->append(if_());
            } else {
                break;
            }
        }
        true_or_die(t->type == EOF_TOKEN || t->type == R_CURLY,
            "invalid characters \"" + t->contents +
            "\" found in statement block");
        return statements;
    }

    Symbol accept_variable(SymbolType direction) {
        true_or_die(direction == INPUT || direction == OUTPUT,
            "variable direction must be INPUT or OUTPUT");
        SymbolTable::iterator symbol = s.find(t->contents);
        true_or_die(symbol != s.end(), "symbol " + t->contents + " not found");
        true_or_die(symbol->second.type == direction ||
                    symbol->second.type == REG,
            "attempting " +
            string(direction == OUTPUT ? "assignment to" : "reading from a")
            + " variable \"" + symbol->first + "\" which is not an " +
            string(direction == OUTPUT ? "OUTPUT" : "INPUT")
            + " or REG");
        t++;
        return symbol->second;
    }

    void ignore_newlines() {
        while (t->type == NEWLINE) {
            t++;
        }
    }

    void accept_colon() {
        true_or_die(t->type == COLON,
                "expected colon but found " + t->contents);
        t++;
    }

    void accept_assign() {
        true_or_die(t->type == ASSIGN,
                "expected assignment but found " + t->contents);
        t++;
    }

    // find series of expressions, push them into an expression block for
    // scheduling
    Statements* expressions() {
        ExpressionBlock* block = new ExpressionBlock();
        while (t->type == VAR) {
            // result
            Expression e;
            e.result = accept_variable(OUTPUT);
            // =
            accept_assign();
            // first arg
            e.add_argument(accept_variable(INPUT));
            // Just an assignment
            if (t->type == VAR
             || t->type == IF
             || t->type == WHILE
             || t->type == DO
             || t->type == EOF_TOKEN
             || t->type == R_CURLY) {
                e.op = ASSIGN;
                block->add_expression(e);
                continue;
            }
            // operator
            true_or_die(t->is_op(), "non-operator " + t->contents +
                    " in expression");
            e.op = t->type;
            t++;
            // second arg
            e.add_argument(accept_variable(INPUT));
            // ternary operator
            if (e.op == QMARK && t->type == COLON) {
                t++;
                // third arg
                e.add_argument(accept_variable(INPUT));
            }
            block->add_expression(e);
        }
        return block;
    }

    Statements* while_() {
        While* w =  new While();
        return w;
    }

    Statements* do_() {
        return new Do();
    }

    Statements* if_() {
        If* i = new If();
        true_or_die(t->type == IF, "expected 'if' but found " + t->contents);
        t++;

        true_or_die(t->type == L_PAREN, "expected '(' but found " + t->contents);
        t++;

        true_or_die(t->type == VAR, "expected a variable but found " + t->contents);
        i->condition = accept_variable(INPUT);

        true_or_die(t->type == R_PAREN, "expected ')' but found " + t->contents);
        t++;

        true_or_die(t->type == L_CURLY, "expected '{' but found " + t->contents);
        t++;

        i->append(statements());

        true_or_die(t->type == R_CURLY, "expected '}' but found " + t->contents);
        t++;

        return i;
    }

    TokenStream t;
    SymbolTable s;
};

int main(int argc, char** argv) {
	if( argc != 4 ) {
		cerr << '\n' << "Usage: " << argv[0] <<
            " -ns cfile verilogfile\n\n";
		return -1;
	}
    string verilog_name(argv[3]);
    fstream c_file(argv[2]);
    true_or_die(c_file.is_open(), "the c file could not be opened");

    Parser p(c_file);
    Program program = p.program();
    c_file.close();

    NoSchedule no_schedule;
    program.generate_fsm(cout, no_schedule);

	return 0;
}

