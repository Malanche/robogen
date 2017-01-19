/*
 * Grammar.cpp
 *
 *  Created on: Nov 8, 2016
 *      Author: lis
 */

#include <iostream>
#include <sstream>
#include <math.h>
#include <queue>

//#define DEBUG_GRAMMAR

#include "evolution/representation/Grammar.h"

using namespace robogen;

Grammar::Rule::Rule(int iterations, boost::shared_ptr<SubRobotRepresentation> predecessor,
                 boost::shared_ptr<SubRobotRepresentation> successor,
                 boost::shared_ptr<effectMap> deleteMap,
                 boost::shared_ptr<effectMap> buildMap,
                 std::vector<buildStep> insertions,
	             std::vector< std::string > deletions,
                 std::vector<paramMutationMap> paramMutations){
	
	this->iterations_ = iterations;
	this->successor_ = successor;
	this->predecessor_ = predecessor;
	this->deleteMap_ = deleteMap;
	this->buildMap_ = buildMap;

	this->deletions_ = deletions;
	this->insertions_ = insertions;
	this->paramMutations_ = paramMutations;
}

int Grammar::Rule::getNumIterations(){
	return this->iterations_;
}

bool generateCloneMap(boost::shared_ptr<PartRepresentation> ruleNode, boost::shared_ptr<PartRepresentation> robotNode,
						boost::shared_ptr<Grammar::Rule::effectMap> theMap, bool isRoot=true){
	//We check if the main node is the core, if yes, we return false.
	if(robotNode->getType()=="ParametricPrismCore"){
		return false;
	}
	
	//if we have a root node, we add the id's to the map
	if(isRoot){
		if(theMap!=NULL){
			(*theMap)[ruleNode->getId()] = robotNode->getParent()->getId();
		}
		//If it is root, this means we have been passed the pointer to the core of the successor
		return generateCloneMap(ruleNode->getChild(0), robotNode, theMap, false);
	}

	//Comparison part
	/**
	 * HERE is where you define your characters (in L-systems).
	 * in the following if, you can select what variables of the node can be selected
	 * as relevant variables.
	 */
	//Relevant variables: Type and groupID
	if( ( ruleNode->getType()!=robotNode->getType() ) || ( ruleNode->getGroupId()!=robotNode->getGroupId() ) ){
		return false;
	} else {
		if(theMap!=NULL){
			(*theMap)[ruleNode->getId()] = robotNode->getId();
		}
		for(int i=0;i < ruleNode->getArity(); i++){
			if(ruleNode->getChild(i)!=NULL){
				boost::shared_ptr<PartRepresentation> child = robotNode->getChild(ruleNode->getChild(i)->getPosition());
				if(child==NULL){
					return false;
				} else {
					if(!generateCloneMap(ruleNode->getChild(i), child, theMap, false)){
						return false;
					}
				}
			}
		}
		//all kids matched, so it is true
		return true;
	}
}

bool Grammar::Rule::matchesPredecessor(boost::shared_ptr<PartRepresentation> candidate){
	boost::shared_ptr<PartRepresentation> tree = this->predecessor_->getTree();
	return generateCloneMap(tree->getChild(0), candidate, boost::shared_ptr<Grammar::Rule::effectMap>(), true);
}

bool Grammar::Rule::mutate(boost::random::mt19937 &rng, boost::shared_ptr<EvolverConfiguration> conf){
	boost::random::bernoulli_distribution<double> dist(0.5);

	if(dist(rng)){
		if(this->paramMutations_.size()==0){
			return false;
		}
		boost::random::uniform_int_distribution<> pieceDist(0,this->paramMutations_.size()-1);
		int toSelect = pieceDist(rng);

		std::vector<double> newParams = this->paramMutations_.at(toSelect).second;

		boost::random::uniform_01<double> paramDist;

		for(int i=0; i<newParams.size(); i++){
			newParams.at(i) = paramDist(rng);
		}
		this->paramMutations_.at(toSelect).second = newParams;
		return true;
	} else { //Deletion or insertion step mutation

		//We first build a copy of the predecessor to advance it to the last
		//step of the deletions

		int attempt=0;

		boost::shared_ptr<SubRobotRepresentation> successor = boost::shared_ptr<SubRobotRepresentation>(new SubRobotRepresentation(*this->predecessor_.get()));
		SubRobotRepresentation::IdPartMap parts = successor->getBody();

		for(int i=0; i< this->deletions_.size(); i++){
			successor->removePart(deletions_.at(i), false);
		}

		parts = successor->getBody();

		if(dist(rng)){ //Mutation in the deletion steps
			if(parts.size()==1){ //We only have the core already
				return false;
			}

			std::string target;
			while(attempt<100){
				//We generate a uniform random distribution for all pieces in the predecessor.
				boost::random::uniform_int_distribution<> dist(0, parts.size() - 1 -1);
				//We get an iterator and we point it to the beginning of the parts.
				SubRobotRepresentation::IdPartMap::const_iterator partToRemove =
						parts.begin();
				//We advance the pointer by a random number of steps, skipping the core
				std::advance(partToRemove, dist(rng)+1);

				//We try to remove the part of the subrobot
				target = partToRemove->first;
				bool success = successor->removePart(target, false);

				if(success){
					deletions_.push_back(target);
					break;
				}
				attempt++;
			}
			if(attempt==100){
				return false;
			} else {

				std::queue<std::string> toEliminate;
				toEliminate.push(target);
				while(!toEliminate.empty()){
					std::string current = toEliminate.front();
					toEliminate.pop();

					for(int i =0; i < this->insertions_.size(); i++){
						if(current == this->insertions_.at(i).parentPartId){
							toEliminate.push(this->insertions_.at(i).newPartId);
							this->insertions_.erase(this->insertions_.begin()+i);
							i--;
						}
					}
				}
			}

		} else { //Mutation in the Insertion steps
			//APPEND TIMEEEEEEE

			int offSet = 0;

			if(successor->getBody().size()>1){
				offSet = 1;
			}

			boost::random::uniform_int_distribution<> dist(offSet, successor->getBody().size() - 1);

			SubRobotRepresentation::IdPartMap::const_iterator parent;


			boost::shared_ptr<PartRepresentation> parentPart;

			// find a parent with arity > 0 , give up after 100 attempts
			int attempt = 0;
			do {
				parent = successor->getBody().begin();
				std::advance(parent, dist(rng));
				parentPart = parent->second.lock();
				attempt++;
			} while (parentPart->getArity() == 0 && attempt<100);

			if(attempt==100){
				//std::cout << "Predecessor impossible to grow." << std::endl;
				return false;
			}

			boost::random::uniform_int_distribution<> distType(0,
				conf->allowedBodyPartTypes.size() - 1);
			char type = conf->allowedBodyPartTypes[distType(rng)];

			// Randomly generate node orientation
			boost::random::uniform_int_distribution<> orientationDist(0, 3);
			unsigned int curOrientation = orientationDist(rng);

			// Randomly generate parameters
			unsigned int nParams = PART_TYPE_PARAM_COUNT_MAP.at(PART_TYPE_MAP.at(type));
			std::vector<double> parameters;
			boost::random::uniform_01<double> paramDist;
			for (unsigned int i = 0; i < nParams; ++i) {
				parameters.push_back(paramDist(rng));
			}

			// Create the new part
			boost::shared_ptr<PartRepresentation> newPart = PartRepresentation::create(
					type, "", curOrientation, parameters);

			//Create a backup Part, before it got inserted in the predecessor
			boost::shared_ptr<PartRepresentation> backupNewPart = PartRepresentation::create(
					type, "", curOrientation, parameters);

			unsigned int newPartSlot = 0;

			if (newPart->getArity() > 0) {
				// Generate a random slot in the new node, if it has arity > 0
				boost::random::uniform_int_distribution<> distNewPartSlot(0,
						newPart->getArity() - 1);
				newPartSlot = distNewPartSlot(rng);
			}
			// otherwise just keep it at 0... inserting part will fail if arity is 0 and
			// there were previously parts attached to the parent's chosen slot

			boost::random::bernoulli_distribution<double> oscillatorNeuronDist_(conf->pOscillatorNeuron);

			int mNType = oscillatorNeuronDist_(rng) ? NeuronRepresentation::OSCILLATOR :
							NeuronRepresentation::SIGMOID;

			// Sample a random slot
			boost::random::uniform_int_distribution<> slotDist(0,
														parentPart->getArity() - 1);
			unsigned int parentSlot = slotDist(rng);

			if(successor->getBody().size()==1){
				parentSlot=0;
			}

			bool succeed = successor->insertPart(parent->first, parentSlot, newPart, newPartSlot,
					mNType, false);

			//If the insertion was successfull, we keep this step in the insertions.
			if(succeed){
				buildStep tmpStep;

				tmpStep.parentPartId = parent->first;
				tmpStep.parentPartSlot = parentSlot;
				tmpStep.newPart = backupNewPart;
				tmpStep.newPartId = newPart->getId();
				tmpStep.newPartSlot = newPartSlot;
				tmpStep.motorNeuronType = mNType;

				this->insertions_.push_back(tmpStep);
			}
		}
	}
	return true;
}

boost::shared_ptr<SubRobotRepresentation> Grammar::Rule::applyRuleTo(boost::shared_ptr<SubRobotRepresentation> robot){
	
	boost::shared_ptr<SubRobotRepresentation> finalBot = boost::shared_ptr<SubRobotRepresentation>(new SubRobotRepresentation(*robot.get()));

	//We will use this robot map in any case. Always, remember please.
	typedef SubRobotRepresentation::IdPartMap::iterator it_type;
	SubRobotRepresentation::IdPartMap robotMap = robot->getBody();

	for(it_type iterator = robotMap.begin(); iterator != robotMap.end(); iterator++) {

		#ifdef DEBUG_GRAMMAR
		std::cout << "Dealing with part " << iterator->first << " of type " << iterator->second.lock()->getType() << std::endl;
		#endif
		
		boost::shared_ptr<SubRobotRepresentation> successor = boost::shared_ptr<SubRobotRepresentation>(new SubRobotRepresentation(*this->predecessor_.get()));
		boost::shared_ptr<Rule::effectMap> tmpMap = boost::shared_ptr<Rule::effectMap>(new Rule::effectMap(*this->buildMap_.get()));

		SubRobotRepresentation::IdPartMap currentMap = finalBot->getBody();

		#ifdef DEBUG_GRAMMAR
		std::cout << "Ready to start checking if the part matches..." << std::endl;
		#endif

		if(currentMap.find(iterator->first)!=currentMap.end()){
			if(generateCloneMap(this->predecessor_->getTree(),finalBot->getBody().at(iterator->first).lock(), tmpMap)){ //If there is no match, we are still ok
				#ifdef DEBUG_GRAMMAR
				std::cout << "-->This part has a match" << std::endl;
				std::cout << "-->Starting deletion steps.." << std::endl;
				#endif

				for(int i=0; i< this->deletions_.size(); i++){
					#ifdef DEBUG_GRAMMAR
					std::cout << i+1 << "..." << std::endl;
					#endif
					successor->removePart(deletions_.at(i), false);
					bool success = finalBot->removePart(tmpMap->at(deletions_.at(i)), false);
					if(!success){
						return boost::shared_ptr<SubRobotRepresentation>();
					}
				}
				#ifdef DEBUG_GRAMMAR
				std::cout << "Current robot:" << std::endl << finalBot->toString() << std::endl;	
				std::cout << "-->Deletion steps concluded." << std::endl << "-->Starting Insertion steps.." << std::endl;
				#endif

				for(int i=0; i< this->insertions_.size(); i++){
					buildStep tmpStep = this->insertions_.at(i);

					boost::shared_ptr<PartRepresentation> newPart = boost::shared_ptr<PartRepresentation>(new PartRepresentation(*tmpStep.newPart.get()));
					boost::shared_ptr<PartRepresentation> newPart2 = boost::shared_ptr<PartRepresentation>(new PartRepresentation(*tmpStep.newPart.get()));

					successor->insertPart(tmpStep.parentPartId,
							tmpStep.parentPartSlot,
							newPart,
							tmpStep.newPartSlot,
							tmpStep.motorNeuronType,
							false);

					#ifdef DEBUG_GRAMMAR
					std::cout << "-->Inserting " << newPart->getType() << " equivalent.." << std::endl;
					std::cout << "(Trying to insert at " << (*tmpMap)[tmpStep.parentPartId] << ")" << std::endl;
					#endif

					bool success = finalBot->insertPart((*tmpMap)[tmpStep.parentPartId],
							tmpStep.parentPartSlot,
							newPart2,
							tmpStep.newPartSlot,
							tmpStep.motorNeuronType,
							false);

					if(!success){
						return boost::shared_ptr<SubRobotRepresentation>();
					}

					(*tmpMap)[successor->getNodeById(tmpStep.parentPartId)->getChild(tmpStep.parentPartSlot)->getId()] = newPart2->getId();
				}

				#ifdef DEBUG_GRAMMAR
				std::cout << "-->Insertion steps concluded" << std::endl;
				std::cout << "Robot after insertions:" <<std::endl << finalBot->toString() << std::endl << "Starting Parameter Mutation steps..." << std::endl;
				std::cout << "Expecting " << this->paramMutations_.size() << " param mutations." << std::endl;
				#endif

				typedef SubRobotRepresentation::IdPartMap::iterator it_type;
				SubRobotRepresentation::IdPartMap bot = finalBot->getBody();

				for(int i=0; i< this->paramMutations_.size(); i++){
					std::vector<double> params = bot.at((*tmpMap)[this->paramMutations_.at(i).first]).lock()->getParams();

					#ifdef DEBUG_GRAMMAR
					std::cout << i+1 << " mutation... to " << this->paramMutations_.at(i).first << std::endl;
					std::cout << "First vector holds " << this->paramMutations_.at(i).second.size() << " entrances." << std::endl;
					std::cout << "Target vector holds " << this->paramMutations_.at(i).second.size() << " entrances." << std::endl;
					#endif

					for(int j=0; j<params.size(); j++){
						params.at(j) = this->paramMutations_.at(i).second.at(j);
					}
				}
				#ifdef DEBUG_GRAMMAR
				std::cout << "-->Parameter Mutation steps concluded." << std::endl;
				#endif
			} else {
				#ifdef DEBUG_GRAMMAR
				std::cout << "Part didn't match." << std::endl;
				#endif
			}
			#ifdef DEBUG_GRAMMAR
			std::cout << "-->Successful iteration" << std::endl;
			#endif
		}
	}

	return finalBot;
}

boost::shared_ptr<SubRobotRepresentation> Grammar::Rule::getSuccessor(void){
	return this->successor_;
}

boost::shared_ptr<SubRobotRepresentation> Grammar::Rule::getPredecessor(void){
	return this->predecessor_;
}

int Grammar::getNumberOfRules(){
	return this->rules_.size();
}

boost::shared_ptr<Grammar::Rule> Grammar::getRule(int id){
	return this->rules_.at(id);
}

bool Grammar::swapRules(int rule1, int rule2){

}

void Grammar::popLastRule(void){
	if(this->rules_.size()>0){
		this->rules_.pop_back();
	}
}

bool Grammar::popRuleAt(int indx){
	if(indx >= this->rules_.size()){
		return false;
	} else {
		this->rules_.erase(this->rules_.begin() + indx);
	}
}

Grammar::Grammar(boost::shared_ptr<SubRobotRepresentation> axiom){
	//Make the pointer point to a new empty subrobot representation
    this->axiom_.reset(new SubRobotRepresentation());
	//Deep copy the passed axiom.
	*this->axiom_ = *axiom;
}

Grammar::Grammar(boost::shared_ptr<SubRobotRepresentation> axiom, std::vector< boost::shared_ptr<Rule> > rules){
	//Make the pointer point to a new empty subrobot representation
    this->axiom_.reset(new SubRobotRepresentation());
	//Deep copy the passed axiom.
	*this->axiom_ = *axiom;
	this->rules_ = rules;
}

boost::shared_ptr<SubRobotRepresentation> Grammar::getAxiom(void){
	return this->axiom_;
}

std::vector< boost::shared_ptr<Grammar::Rule> > Grammar::getAllRules(){
	return this->rules_;
}

boost::shared_ptr<SubRobotRepresentation> Grammar::buildTree(void){

	#ifdef DEBUG_GRAMMAR
	std::cout << "Building a robot, starting from axiom:" << std::endl;
	std::cout << this->axiom_->toString() << std::endl;
	#endif

	//emtpy pointer for the final build
	boost::shared_ptr<SubRobotRepresentation> final;

	final.reset(new SubRobotRepresentation());
	//We start with the axiom
	*final = *this->axiom_;

	typedef SubRobotRepresentation::IdPartMap::iterator it_type;
	SubRobotRepresentation::IdPartMap bot = final->getBody();

	int nRules = this->rules_.size();

	#ifdef DEBUG_GRAMMAR
	std::cout << "The robot has " << nRules << " rules" << std::endl;
	#endif

	//Iterate over every rule
	for(int r=0;r<nRules;r++){
		int nIter = this->rules_.at(r)->getNumIterations();


		#ifdef DEBUG_GRAMMAR
		std::cout << "Rule " << r+1 << " has the following shape:" << std::endl;

		std::cout << "Predecessor:" << std::endl;
		std::cout << this->rules_.at(r)->getPredecessor()->toString() << std::endl;
		std::cout << "Successor:" << std::endl;
		std::cout << this->rules_.at(r)->getSuccessor()->toString() << std::endl;

		std::cout << "Rule " << r+1 << " has " << nIter << " iterations." << std::endl;
		#endif

		//Repeat each rule as many times as required
		for(int n=0;n<nIter;n++){

			#ifdef DEBUG_GRAMMAR
			std::cout << "Iteration number " << n+1 << std::endl;
			#endif

			final = this->rules_.at(r)->applyRuleTo(final);

			if(final==NULL){
				//final = boost::shared_ptr<SubRobotRepresentation>(new SubRobotRepresentation(*this->axiom_.get()));
				//std::cout << final->toString() << std::endl;
				return final;
			}
		}
	}
	return final;
}

bool Grammar::addRule(boost::shared_ptr<Grammar::Rule> newRule){
	this->rules_.push_back(newRule);
	return true;
}