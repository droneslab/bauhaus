mod imutests {
    use std::collections::BTreeMap;


    #[test]
    fn test_range() {
        let mut map = BTreeMap::new();
        map.insert(3, "a");
        map.insert(5, "b");
        map.insert(8, "c");
        assert_eq!(Some((&3, &"a")), map.range(3..).next());
        assert_eq!(Some((&5, &"b")), map.range(4..).next());

    }
}