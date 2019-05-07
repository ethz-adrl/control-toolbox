#ifndef CPPAD_CG_LANGUAGE_MATHML_INDEX_PATTERNS_INCLUDED
#define CPPAD_CG_LANGUAGE_MATHML_INDEX_PATTERNS_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2015 Ciengis
 *
 *  CppADCodeGen is distributed under multiple licenses:
 *
 *   - Eclipse Public License Version 1.0 (EPL1), and
 *   - GNU General Public License Version 3 (GPL3).
 *
 *  EPL1 terms and conditions can be found in the file "epl-v10.txt", while
 *  terms and conditions for the GPL3 can be found in the file "gpl3.txt".
 * ----------------------------------------------------------------------------
 * Author: Joao Leal
 */

namespace CppAD {
namespace cg {

template<class Base>
inline void LanguageMathML<Base>::generateNames4RandomIndexPatterns(const std::set<RandomIndexPattern*>& randomPatterns) {
    std::ostringstream os;

    std::set<std::string> usedNames;

    // save existing names so that they are not to overridden 
    // (independent variable names might have already used them)
    for (RandomIndexPattern* ip : randomPatterns) {
        if (!ip->getName().empty()) {
            usedNames.insert(ip->getName());
        }
    }

    // create new names for the index pattern arrays without a name
    size_t c = 0;
    for (RandomIndexPattern* ip : randomPatterns) {
        if (ip->getName().empty()) {
            // new name required
            std::string name;
            do {
                os << _C_STATIC_INDEX_ARRAY << c;
                name = os.str();
                os.str("");
                c++;
            } while (usedNames.find(name) != usedNames.end());

            ip->setName("<mi>" + name + "</mi>");
        }
    }

}

template<class Base>
inline void LanguageMathML<Base>::printRandomIndexPatternDeclaration(std::ostringstream& os,
                                                                     const std::string& indentation,
                                                                     const std::set<RandomIndexPattern*>& randomPatterns) {
    for (RandomIndexPattern* ip : randomPatterns) {
        if (ip->getType() == IndexPatternType::Random1D) {
            /**
             * 1D
             */
            Random1DIndexPattern* ip1 = static_cast<Random1DIndexPattern*> (ip);
            const std::map<size_t, size_t>& x2y = ip1->getValues();

            std::vector<size_t> y(x2y.rbegin()->first + 1);
            for (const std::pair<size_t, size_t>& p : x2y)
                y[p.first] = p.second;

            os << indentation;
            printStaticIndexArray(os, ip->getName(), y);
        } else {
            CPPADCG_ASSERT_UNKNOWN(ip->getType() == IndexPatternType::Random2D);
            /**
             * 2D
             */
            Random2DIndexPattern* ip2 = static_cast<Random2DIndexPattern*> (ip);
            os << indentation;
            printStaticIndexMatrix(os, ip->getName(), ip2->getValues());
        }
    }
}

template<class Base>
void LanguageMathML<Base>::printStaticIndexArray(std::ostringstream& os,
                                                 const std::string& name,
                                                 const std::vector<size_t>& values) {
    os << name << " <mo>=</mo> <mfenced open='[' close=']' separators=','>";
    for (size_t i = 0; i < values.size(); i++) {
        os << "<mn>" << values[i] << "</mn>";
    }
    os << "</mfenced>" << _endEq << " <!-- size: " << values.size() << " -->" << _endline;
}

template<class Base>
void LanguageMathML<Base>::printStaticIndexMatrix(std::ostringstream& os,
                                                  const std::string& name,
                                                  const std::map<size_t, std::map<size_t, size_t> >& values) {
    /**
     * TODO
     */
    size_t m = 0;
    size_t n = 0;

    std::map<size_t, std::map<size_t, size_t> >::const_iterator it;
    std::map<size_t, size_t>::const_iterator ity2z;

    if (!values.empty()) {
        m = values.rbegin()->first + 1;

        for (it = values.begin(); it != values.end(); ++it) {
            if (!it->second.empty())
                n = std::max(n, it->second.rbegin()->first + 1);
        }
    }

    os << name << "<mo>=</mo><mtable>";
    size_t x = 0;
    for (it = values.begin(); it != values.end(); ++it) {
        if (it->first != x) {
            while (it->first != x) {
                os << "<mtr><mtd/></mtr>";
                x++;
            }
        }

        os << "<mtr>";
        size_t y = 0;
        for (ity2z = it->second.begin(); ity2z != it->second.end(); ++ity2z) {
            if (ity2z->first != y) {
                while (ity2z->first != y) {
                    os << "<mtd><mn>0</mn></mtd>";
                    y++;
                }
            }

            os << "<mtd><mn>" << ity2z->second << "</mn></mtd>";


            y++;
        }
        os << "</mtr>";

        x++;
    }
    os << "</mtable>" << _endEq << "<!-- size: " << m << " x " << n << " -->" << _endline;
}

template<class Base>
inline void LanguageMathML<Base>::indexPattern2String(std::ostream& os,
                                                      const IndexPattern& ip,
                                                      const OperationNode<Base>& index) {
    indexPattern2String(os, ip,{&index});
}

template<class Base>
inline void LanguageMathML<Base>::indexPattern2String(std::ostream& os,
                                                      const IndexPattern& ip,
                                                      const std::vector<const OperationNode<Base>*>& indexes) {
    std::stringstream ss;
    switch (ip.getType()) {
        case IndexPatternType::Linear: // y = x * a + b
        {
            CPPADCG_ASSERT_KNOWN(indexes.size() == 1, "Invalid number of indexes");
            const LinearIndexPattern& lip = static_cast<const LinearIndexPattern&> (ip);
            linearIndexPattern2String(os, lip, *indexes[0]);
            return;
        }
        case IndexPatternType::Sectioned:
        {
            CPPADCG_ASSERT_KNOWN(indexes.size() == 1, "Invalid number of indexes");
            const SectionedIndexPattern* lip = static_cast<const SectionedIndexPattern*> (&ip);
            const std::map<size_t, IndexPattern*>& sections = lip->getLinearSections();
            size_t sSize = sections.size();
            CPPADCG_ASSERT_UNKNOWN(sSize > 1);

            std::map<size_t, IndexPattern*>::const_iterator its = sections.begin();
            for (size_t s = 0; s < sSize - 1; s++) {
                const IndexPattern* lp = its->second;
                ++its;
                size_t xStart = its->first;

                os << "<mfenced><mrow><mi>" << (*indexes[0]->getName()) << "</mi> <mo>&lt;</mo> <mn>" << xStart << "</mn></mrow></mfenced><mo>?</mo> ";
                indexPattern2String(os, *lp, *indexes[0]);
                os << "<mo>:</mo> ";
            }
            indexPattern2String(os, *its->second, *indexes[0]);

            return;
        }

        case IndexPatternType::Plane2D: // y = f(x) + f(z)
        {
            CPPADCG_ASSERT_KNOWN(indexes.size() >= 1, "Invalid number of indexes");
            const Plane2DIndexPattern& pip = static_cast<const Plane2DIndexPattern&> (ip);
            bool useParens = pip.getPattern1() != nullptr && pip.getPattern2() != nullptr;

            if (useParens) os << "<mfenced><mrow>";

            if (pip.getPattern1() != nullptr)
                indexPattern2String(os, *pip.getPattern1(), *indexes[0]);

            if (useParens) os << "</mrow></mfenced> <mo>+</mo> <mfenced><mrow>";

            if (pip.getPattern2() != nullptr)
                indexPattern2String(os, *pip.getPattern2(), *indexes.back());

            if (useParens) os << "</mrow></mfenced>";

            return;
        }
        case IndexPatternType::Random1D:
        {
            CPPADCG_ASSERT_KNOWN(indexes.size() == 1, "Invalid number of indexes");
            const Random1DIndexPattern& rip = static_cast<const Random1DIndexPattern&> (ip);
            CPPADCG_ASSERT_KNOWN(!rip.getName().empty(), "Invalid name for array");
            os << rip.getName() << "<mfenced open='[' close=']'><mi>" << (*indexes[0]->getName()) << "</mi></mfenced>";
            return;
        }
        case IndexPatternType::Random2D:
        {
            CPPADCG_ASSERT_KNOWN(indexes.size() == 2, "Invalid number of indexes");
            const Random2DIndexPattern& rip = static_cast<const Random2DIndexPattern&> (ip);
            CPPADCG_ASSERT_KNOWN(!rip.getName().empty(), "Invalid name for array");
            os << rip.getName() <<
                    "<mfenced open='[' close=']'><mrow><mi>" << (*indexes[0]->getName()) << "</mi></mrow></mfenced>"
                    "<mfenced open='[' close=']'><mrow><mi>" << (*indexes[1]->getName()) << "</mi></mrow></mfenced>";
            return;
        }
        default:
            CPPADCG_ASSERT_UNKNOWN(false); // should never reach this
            return;
    }
}

template<class Base>
inline void LanguageMathML<Base>::linearIndexPattern2String(std::ostream& os,
                                                            const LinearIndexPattern& lip,
                                                            const OperationNode<Base>& index) {
    long dy = lip.getLinearSlopeDy();
    long dx = lip.getLinearSlopeDx();
    long b = lip.getLinearConstantTerm();
    long xOffset = lip.getXOffset();

    if (dy != 0) {
        if (xOffset != 0) {
            os << "<mfenced><mrow>";
        }
        os << "<mi>" << (*index.getName()) << "</mi>";
        if (xOffset != 0) {
            os << " <mo>-</mo><mn>" << xOffset << "</mn></mrow></mfenced>";
        }

        if (dx != 1) {
            os << "<mo>/</mo><mn>" << dx << "</mn>";
        }
        if (dy != 1) {
            os << "<mo>\\times</mo><mn>" << dy << "</mn>";
        }
    } else if (b == 0) {
        os << "<mn>0</mn>"; // when dy == 0 and b == 0
    }

    if (b != 0) {
        if (dy != 0)
            os << "<mo>+</mo>";
        os << "<mn>" << b << "</mn>";
    }

}

} // END cg namespace
} // END CppAD namespace

#endif