#include <fan/graphics/opengl/3D/objects/fms.h>
#include <iomanip>

#include <stdint.h>

#include <cctype>

std::vector<std::vector<std::string>> bone_names_default = {
  {"Hips", "Torso"},
  {"Right_Up_Leg", "Upper_Leg_R", "RightUpLeg", "Upper_Leg.R", "R_UpperLeg"},
  {"Lower_Leg_R", "Lower_Leg.R", "Right_knee", "R_LowerLeg"},
  {"Right_Foot", "Foot_R", "RightFoot", "Foot.R", "Right_ankle", "R_Foot"},
  {"Right_Toe_Base", "RightToeBase", "Right_toe", "R_ToeBase"},
  {"Right_Toe_End"},
  {"Spine"},
  {"Spine1", "Chest", "UpperChest"},
  {"Spine2"},
  {"Neck"},
  {"Head"},
  {"Head_Top_End"},
  {"Left_Shoulder", "Upper_Arm_L", "L_Shoulder"},
  {"Left_Arm", "L_UpperArm"},
  {"Left_Fore_Arm", "Lower_Arm_L", "Left_elbow", "L_LowerArm"},
  {"Left_Hand", "Hand_L", "Left_wrist", "L_Hand"},
  {"Left_Hand_Thumb1", "Thumb0_L"},
  {"Left_Hand_Thumb2", "Thumb1_L"},
  {"Left_Hand_Thumb3", "Thumb2_L"},
  {"Left_Hand_Thumb4", "Thumb2Tip_L"},
  {"Left_Hand_Index1", "IndexFinger1_L"},
  {"Left_Hand_Index2", "IndexFinger2_L"},
  {"Left_Hand_Index3", "IndexFinger3_L"},
  {"Left_Hand_Index4", "IndexFinger3Tip_L"},
  {"Left_Hand_Middle1", "MiddleFinger1_L"},
  {"Left_Hand_Middle2", "MiddleFinger2_L"},
  {"Left_Hand_Middle3", "MiddleFinger3_L"},
  {"Left_Hand_Middle4", "MiddleFinger3Tip_L"},
  {"Left_Hand_Ring1", "RingFinger1_L"},
  {"Left_Hand_Ring2", "RingFinger2_L"},
  {"Left_Hand_Ring3", "RingFinger3_L"},
  {"Left_Hand_Ring4", "RingFinger3Tip_L"},
  {"Left_Hand_Pinky1", "LittleFinger1_L"},
  {"Left_Hand_Pinky2", "LittleFinger2_L"},
  {"Left_Hand_Pinky3", "LittleFinger3_L"},
  {"Left_Hand_Pinky4", "LittleFinger3Tip_L"},
  {"Right_Shoulder", "Upper_Arm_R", "R_Shoulder"},
  {"Right_Arm", "R_UpperArm"},
  {"Right_Fore_Arm", "Lower_Arm_R", "Right_elbow", "R_LowerArm"},
  {"Right_Hand", "Hand_R", "Right_wrist", "R_Hand"},
  {"Right_Hand_Thumb1", "Thumb0_R"},
  {"Right_Hand_Thumb2", "Thumb1_R"},
  {"Right_Hand_Thumb3", "Thumb2_R"},
  {"Right_Hand_Thumb4", "Thumb2Tip_R"},
  {"Right_Hand_Index1", "IndexFinger1_R"},
  {"Right_Hand_Index2", "IndexFinger2_R"},
  {"Right_Hand_Index3", "IndexFinger3_R"},
  {"Right_Hand_Index4", "IndexFinger3Tip_R"},
  {"Right_Hand_Middle1", "MiddleFinger1_R"},
  {"Right_Hand_Middle2", "MiddleFinger2_R"},
  {"Right_Hand_Middle3", "MiddleFinger3_R"},
  {"Right_Hand_Middle4", "MiddleFinger3Tip_R"},
  {"Right_Hand_Ring1", "RingFinger1_R"},
  {"Right_Hand_Ring2", "RingFinger2_R"},
  {"Right_Hand_Ring3", "RingFinger3_R"},
  {"Right_Hand_Ring4", "RingFinger3Tip_R"},
  {"Right_Hand_Pinky1", "LittleFinger1_R"},
  {"Right_Hand_Pinky2", "LittleFinger2_R"},
  {"Right_Hand_Pinky3", "LittleFinger3_R"},
  {"Right_Hand_Pinky4", "LittleFinger3Tip_R"},
  {"Left_Up_Leg", "Upper_Leg_L", "LeftUpLeg", "Upper_Leg.L", "L_UpperLeg"},
  {"Lower_Leg_L", "Lower_Leg.L", "Left_knee", "L_LowerLeg"},
  {"Left_Foot", "Foot_L", "LeftFoot", "Foot.L", "Left_ankle", "L_Foot"},
  {"Left_Toe_Base", "LeftToeBase", "Left_toe", "L_ToeBase"},
  {"Left_Toe_End"}
};

std::vector<std::vector<std::string>> bone_names_anim = bone_names_default;
std::vector<std::vector<std::string>> bone_names_model = bone_names_default;

uintptr_t get_bone_name_index(auto &iteration, std::string from){
  uintptr_t longest_name_id = (uintptr_t)-1;
  uintptr_t longest_length = 0;
  uintptr_t shortest_length_from = (uintptr_t)-1;

  for(uintptr_t i = 0; i < iteration.size(); i++){
    for(uintptr_t bni1 = 0; bni1 < iteration[i].size(); bni1++){
      std::string bn = iteration[i][bni1];
      for(uintptr_t ip = 0; ip < from.size(); ip++){
        uintptr_t bni = 0;
        auto nfrom = from.substr(ip);
        uintptr_t nfromi = 0;
        while(1){
          if(bni == bn.size()){
            if(
              (bni > longest_length) ||
              (from.size() < shortest_length_from)
            ){
              longest_name_id = i;
              longest_length = bni;
              shortest_length_from = from.size();
            }
            break;
          }

          if(nfromi == nfrom.size()){
            break;
          }

          if(std::tolower(bn[bni]) == std::tolower(nfrom[nfromi])){
            bni++;
            nfromi++;
            continue;
          }
          else if(bn[bni] == '_'){
            bni++;
          }
          else{
            break;
          }
        }
      }
    }
  }

  return longest_name_id;
}

std::string get_model_bone_name(uintptr_t name_index, auto &model){
  std::string longest_name;
  uintptr_t longest_length = 0;
  uintptr_t shortest_length_from = (uintptr_t)-1;

  model.iterate_bones(*model.root_bone, [&](fan_3d::model::bone_t& bone){
    std::string from = bone.name;

    for(uintptr_t bni1 = 0; bni1 < bone_names_model[name_index].size(); bni1++){
      std::string bn = bone_names_model[name_index][bni1];
      for(uintptr_t ip = 0; ip < from.size(); ip++){
        uintptr_t bni = 0;
        auto nfrom = from.substr(ip);
        uintptr_t nfromi = 0;
        while(1){
          if(bni == bn.size()){
            if(
              (from.size() < shortest_length_from) ||
              (from.size() == shortest_length_from && bni > longest_length)
            ){
              longest_name = from;
              longest_length = bni;
              shortest_length_from = from.size();
            }
            break;
          }

          if(nfromi == nfrom.size()){
            break;
          }

          if(std::tolower(bn[bni]) == std::tolower(nfrom[nfromi])){
            bni++;
            nfromi++;
            continue;
          }
          else if(bn[bni] == '_'){
            bni++;
          }
          else{
            break;
          }
        }
      }
    }
  });

  return longest_name;
}

void solve_legs(auto &iterator, auto &vector){
  uint8_t left_leg_meaning = (uint8_t)-1;
  std::vector<std::vector<std::string>> left_leg_vector = {
    {"Left_leg"}
  };
  bool left_leg = false;
  iterator.iterate_bones(*iterator.root_bone, [&](fan_3d::model::bone_t& bone){
    if(get_bone_name_index(left_leg_vector, bone.name) == 0){
      if(left_leg){
        /* multiple left leg? */
        assert(0);
      }
      left_leg = true;
    }
  });
  if(left_leg){
    std::vector<std::vector<std::string>> left_down_leg_vector = {
      {"Lower_Leg_L", "Left_knee"}
    };
    iterator.iterate_bones(*iterator.root_bone, [&](fan_3d::model::bone_t& bone){
      if(get_bone_name_index(left_down_leg_vector, bone.name) == 0){
        if(left_leg_meaning != (uint8_t)-1){
          /* multiple left leg meaning? */
          assert(0);
        }
        left_leg_meaning = 0;
      }
    });
    std::vector<std::vector<std::string>> left_up_leg_vector = {
      {"Left_Up_Leg"}
    };
    iterator.iterate_bones(*iterator.root_bone, [&](fan_3d::model::bone_t& bone){
      if(get_bone_name_index(left_up_leg_vector, bone.name) == 0){
        if(left_leg_meaning != (uint8_t)-1){
          /* multiple left leg meaning? */
          assert(0);
        }
        left_leg_meaning = 1;
      }
    });
  }
  if(left_leg_meaning != (uint8_t)-1){
    uintptr_t index;
    if(left_leg_meaning == 0){
      index = get_bone_name_index(bone_names_default, "Left_Up_Leg");
    }
    else if(left_leg_meaning == 1){
      index = get_bone_name_index(bone_names_default, "Lower_Leg_L");
    }
    else{
      index = (uintptr_t)-1;
    }
    if(index == (uintptr_t)-1){
      /* internal error */
      assert(0);
    }

    vector[index].push_back("Left_leg");
  }

  uint8_t right_leg_meaning = (uint8_t)-1;
  std::vector<std::vector<std::string>> right_leg_vector = {
    {"Right_leg"}
  };
  bool right_leg = false;
  iterator.iterate_bones(*iterator.root_bone, [&](fan_3d::model::bone_t& bone){
    if(get_bone_name_index(right_leg_vector, bone.name) == 0){
      if(right_leg){
        /* multiple right leg? */
        assert(0);
      }
      right_leg = true;
    }
  });
  if(right_leg){
    std::vector<std::vector<std::string>> right_down_leg_vector = {
      {"Lower_Leg_R", "Right_knee"}
    };
    iterator.iterate_bones(*iterator.root_bone, [&](fan_3d::model::bone_t& bone){
      if(get_bone_name_index(right_down_leg_vector, bone.name) == 0){
        if(right_leg_meaning != (uint8_t)-1){
          /* multiple right leg meaning? */
          assert(0);
        }
        right_leg_meaning = 0;
      }
    });
    std::vector<std::vector<std::string>> right_up_leg_vector = {
      {"Right_Up_Leg"}
    };
    iterator.iterate_bones(*iterator.root_bone, [&](fan_3d::model::bone_t& bone){
      if(get_bone_name_index(right_up_leg_vector, bone.name) == 0){
        if(right_leg_meaning != (uint8_t)-1){
          /* multiple right leg meaning? */
          assert(0);
        }
        right_leg_meaning = 1;
      }
    });
  }
  if(right_leg_meaning != (uint8_t)-1){
    uintptr_t index;
    if(right_leg_meaning == 0){
      index = get_bone_name_index(bone_names_default, "Right_Up_Leg");
    }
    else if(right_leg_meaning == 1){
      index = get_bone_name_index(bone_names_default, "Lower_Leg_R");
    }
    else{
      index = (uintptr_t)-1;
    }
    if(index == (uintptr_t)-1){
      /* internal error */
      assert(0);
    }

    vector[index].push_back("Right_leg");
  }
}

void mapper(fan_3d::model::fms_t& model, fan_3d::model::fms_t& anim){
  solve_legs(anim, bone_names_anim);
  solve_legs(model, bone_names_model);

  anim.iterate_bones(*anim.root_bone, [&](fan_3d::model::bone_t& bone) {
    auto bone_name_index = get_bone_name_index(bone_names_anim, bone.name);
    if(bone_name_index == (uintptr_t)-1){
      printf("f \"%s\"\n", bone.name.c_str());
    }
    else{
      auto solved_bone_name = get_model_bone_name(bone_name_index, model);
      if(solved_bone_name.size() == 0){
        printf("nu \"%s\" -> \"%s\"\n", bone.name.c_str(), bone_names_default[bone_name_index][0].c_str());
      }
      else{
        printf("yay \"%s\" -> \"%s\" -> \"%s\"\n", bone.name.c_str(), bone_names_default[bone_name_index][0].c_str(), solved_bone_name.c_str());
      }
    }
  });
}

int main(){
  fan_3d::model::fms_t::properties_t p;
  p.path = "models/final_provence.fbx";
  p.texture_path = "models/textures";
  p.use_cpu = 0;
  fan_3d::model::fms_t model(p);
  p.path = "models/anim0.gltf";
  fan_3d::model::fms_t anim0(p);

  #if 0
  model.iterate_bones(*model.root_bone, [&](fan_3d::model::bone_t& bone) {
    printf("modelname %s\n", bone.name.c_str());
  });
  #endif

  mapper(model, anim0);

  return 0;
}
