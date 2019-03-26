#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/FilterBase.hpp"
#include "lightweight_filtering/Update.hpp"
#include "lightweight_filtering/State.hpp"
#include "lightweight_filtering/Prediction.hpp"
#include "lightweight_filtering/FilterState.hpp"

class State: public LWF::State<LWF::QuaternionElement>{
    
};

class PredictionMeas: public LWF::State<LWF::VectorElement<3>,LWF::VectorElement<3>>{};

template<typename STATE>
class PredictionNoise: public LWF::State<LWF::TH_multiple_elements<LWF::VectorElement<3>,5>>{};

class FilterState: public LWF::FilterState<State,PredictionMeas,PredictionNoise<State>,0>{
public:
    typedef LWF::FilterState<State,PredictionMeas,PredictionNoise<State>,0> Base;
    typedef typename Base::mtState mtState;
};

template<typename FILTERSTATE>
class ImuPrediction: public LWF::Prediction<FILTERSTATE>{
public:
    typedef LWF::Prediction<FILTERSTATE> Base;
    typedef typename Base::mtState mtState;
    typedef typename Base::mtFilterState mtFilterState;
    typedef typename Base::mtMeas mtMeas;
    typedef typename Base::mtNoise mtNoise;
    virtual void evalPrediction(mtState& x, const mtState& previousState, const mtNoise& noise, double dt) const {};
    virtual void jacPreviousState(Eigen::MatrixXd& F, const mtState& previousState, double dt) const {};
    virtual void jacNoise(Eigen::MatrixXd& F, const mtState& previousState, double dt) const {};
};

template<typename STATE> 
class ImgInnovation: public LWF::State<LWF::VectorElement<2>>{};

template<typename STATE>
class ImgUpdateMeasAuxiliary: public LWF::AuxiliaryBase<ImgUpdateMeasAuxiliary<STATE>>{};

template<typename STATE>
class ImgUpdateMeas: public LWF::State<ImgUpdateMeasAuxiliary<STATE>>{};

template<typename STATE>
class ImgUpdateNoise: public LWF::State<LWF::VectorElement<2>>{};

template<typename FILTERSTATE>
class ImgUpdate: public LWF::Update<ImgInnovation<typename FILTERSTATE::mtState>,FILTERSTATE,ImgUpdateMeas<typename FILTERSTATE::mtState>,ImgUpdateNoise<typename FILTERSTATE::mtState>>{
public:
    typedef LWF::Update<ImgInnovation<typename FILTERSTATE::mtState>,FILTERSTATE,ImgUpdateMeas<typename FILTERSTATE::mtState>,ImgUpdateNoise<typename FILTERSTATE::mtState>> Base;
    typedef typename Base::mtState mtState;
    typedef typename Base::mtFilterState mtFilterState;
    typedef typename Base::mtInnovation mtInnovation;
    typedef typename Base::mtMeas mtMeas;
    typedef typename Base::mtNoise mtNoise;
    virtual void evalInnovation(mtInnovation& y, const mtState& state, const mtNoise& noise) const{};
    virtual void jacState(Eigen::MatrixXd& F, const mtState& state) const{};
    virtual void jacNoise(Eigen::MatrixXd& F, const mtState& state) const{};
    
};

template<typename FILTERSTATE>
class RovioFilter:public LWF::FilterBase<ImuPrediction<FILTERSTATE>, ImgUpdate<FILTERSTATE>>{
public:
    typedef LWF::FilterBase<ImuPrediction<FILTERSTATE>, ImgUpdate<FILTERSTATE>> Base;
    using Base::doubleRegister_;
    double try_config;
    void showTest(){      
        doubleRegister_.registerScalar("chamo",try_config);
    }
    
};
                                        
int main(int argc, char* argv[]){
    RovioFilter<FilterState> filter;
    filter.showTest();
    filter.readFromInfo("/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/11_26/chamo.info");
    
    std::cout<<filter.try_config<<std::endl;
    return 0;
}