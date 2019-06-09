//
// Created by Sandra Alfaro on 30/05/18.
//

#ifndef OPENGR_MATCH3PCSBASE_H
#define OPENGR_MATCH3PCSBASE_H

#include <vector>
#include "gr/shared.h"
#include "gr/sampling.h"
#include "gr/algorithms/congruentSetExplorationBase.h"
#include "matchBase.h"

#ifdef TEST_GLOBAL_TIMINGS
#   include "gr/utils/timer.h"
#endif

namespace gr {
    struct Traits3pcs {
        static constexpr int size() { return 3; }
        using Base = std::array<int,3>;
        using Set = std::vector<Base>;
        using Coordinates = std::array<Point3D, 3>;
    };

    /// Class for the computation of the 3PCS algorithm.
    template <typename _TransformVisitor,
              typename _PairFilteringFunctor,  /// <\brief Must implements PairFilterConcept
              template < class, class > typename PairFilteringOptions >
    class Match3pcs : public CongruentSetExplorationBase<Traits3pcs, _TransformVisitor, _PairFilteringFunctor, PairFilteringOptions> {
    public:
      using Traits               = Traits3pcs;
      using PairFilteringFunctor = _PairFilteringFunctor;
      using TransformVisitor     = _TransformVisitor;

      using CongruentBaseType    = typename Traits::Base;
      using Set                  = typename Traits::Set;
      using Coordinates          = typename Traits::Coordinates;

      using MatchBaseType = CongruentSetExplorationBase<Traits3pcs, _TransformVisitor, _PairFilteringFunctor, PairFilteringOptions>;

      using OptionsType = typename MatchBaseType::OptionsType;
      using Scalar      = typename MatchBaseType::Scalar;

        Match3pcs (const OptionsType& options
                , const Utils::Logger& logger);

        virtual ~Match3pcs();

        /// Find all the congruent set similar to the base in the second 3D model (Q).
        /// It could be with a 3 point base or a 4 point base.
        /// \param base use to find the similar points congruent in Q.
        /// \param congruent_set a set of all point congruent found in Q.
        bool generateCongruents (CongruentBaseType& base, Set& congruent_quads) override;

        /// Initializes the data structures and needed values before the match
        /// computation.
        /// @param [in] point_P First input set.
        /// @param [in] point_Q Second input set.
        /// expected to be in the inliers.
        /// This method is called once the internal state of the Base class as been
        /// set.
        void Initialize(const std::vector<Point3D>& /*P*/,
                        const std::vector<Point3D>& /*Q*/) override {}


    };
}

#include "match3pcs.hpp"
#endif //OPENGR_MATCH3PCSBASE_H
